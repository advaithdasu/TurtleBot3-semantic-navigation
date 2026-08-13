"""NVIDIA LocateAnything-3B backend: image + phrase -> bounding boxes.

Needs an Ampere+ NVIDIA GPU, Linux, ~12 GB VRAM, transformers==4.57.1.
Weights are licensed for non-commercial research use only. Inference
follows the model card at https://huggingface.co/nvidia/LocateAnything-3B.
"""

from __future__ import annotations

import io
import time

try:
    from ..boxparse import parse_boxes
except ImportError:
    # server.py runs with the grounding dir as sys.path[0], where
    # `backends` is a top-level package and relative import fails.
    from boxparse import parse_boxes

_MODEL_ID = "nvidia/LocateAnything-3B"
_PROMPT_TEMPLATE = "Locate all instances matching: {query}"


def resolve_dtype(name: str, device: str):
    """Pick the compute dtype, defaulting to whatever the GPU can do.

    The model card specifies bfloat16, which needs Ampere (sm_80) or
    newer for *native* support. On Turing (sm_75 — RTX 20xx, T4) bf16
    still runs, but emulated and without tensor cores, so `auto` prefers
    float16 there: Turing accelerates fp16 natively.

    Do NOT use torch.cuda.is_bf16_supported() for this. Through at least
    torch 2.3 it is written as `capability.major >= 8 OR cuda_version >=
    11`, so on a CUDA 12 build it returns True for *every* card —
    including a 2080 Ti, where bf16 is emulated. It answers "will bf16
    run?", not "is bf16 fast?". Compute capability is the honest check.

    The fp16 preference is a real tradeoff, not a free win: bf16 carries
    fp32's exponent range and fp16 does not, so a bf16-trained model can
    overflow to Inf/NaN in fp16. If boxes come back empty or nonsensical
    on a pre-Ampere card, suspect this before suspecting the prompt —
    `--dtype bfloat16` restores the model card's exact numerics at the
    cost of speed, and `--dtype float32` is slower still but safest.
    """
    import torch

    explicit = {
        "bfloat16": torch.bfloat16,
        "float16": torch.float16,
        "float32": torch.float32,
    }
    if name != "auto":
        return explicit[name]

    if device.startswith("cuda") and torch.cuda.is_available():
        try:
            major, _minor = torch.cuda.get_device_capability(device)
        except (RuntimeError, AssertionError, ValueError):
            major = 0
        if major >= 8:          # Ampere and later: native bf16
            return torch.bfloat16
        return torch.float16
    # CPU: fp16 matmuls are largely unimplemented, so fp32 is the only
    # thing that actually runs (slowly).
    return torch.float32


class LocateAnythingBackend:

    def __init__(self, model_id: str = _MODEL_ID, device: str = "cuda",
                 dtype: str = "auto", max_new_tokens: int = 2048,
                 **_kwargs) -> None:
        self.model_id = model_id
        self.device = device
        self.dtype = dtype
        self.max_new_tokens = max_new_tokens
        self._torch_dtype = None   # resolved in load()
        self._model = None
        self._tokenizer = None
        self._processor = None

    def load(self) -> None:
        import torch
        from transformers import AutoModel, AutoProcessor, AutoTokenizer

        self._torch_dtype = resolve_dtype(self.dtype, self.device)
        if self.dtype == "auto" and self._torch_dtype is torch.float16:
            cap = torch.cuda.get_device_capability(self.device)
            print(f"[locate_anything] compute capability {cap[0]}.{cap[1]} "
                  "(pre-Ampere): bfloat16 would be emulated, using float16. "
                  "Pass --dtype bfloat16 to match the model card exactly.")

        t0 = time.monotonic()
        self._tokenizer = AutoTokenizer.from_pretrained(
            self.model_id, trust_remote_code=True)
        self._processor = AutoProcessor.from_pretrained(
            self.model_id, trust_remote_code=True)
        self._model = (
            AutoModel.from_pretrained(
                self.model_id,
                torch_dtype=self._torch_dtype,
                trust_remote_code=True,
            )
            .to(self.device)
            .eval()
        )
        if self.device.startswith("cuda") and torch.cuda.is_available():
            reserved = torch.cuda.memory_reserved() / 1024 ** 3
            print(f"[locate_anything] {reserved:.1f} GiB reserved after load")
        print(f"[locate_anything] loaded {self.model_id} on {self.device} "
              f"as {self._torch_dtype} in {time.monotonic() - t0:.1f}s")

    def info(self) -> dict:
        return {
            "backend": "locate_anything",
            "device": self.device,
            "model": self.model_id,
            # Reported so a run's numbers can be attributed to a dtype
            # later — an fp16 fallback result is not comparable to a bf16
            # one, and /health is the only place the eval can see it.
            "dtype": str(self._torch_dtype).replace("torch.", "")
                     if self._torch_dtype is not None else self.dtype,
        }

    def ground(self, image_bytes: bytes, query: str) -> list[dict]:
        import torch
        from PIL import Image

        if self._model is None:
            raise RuntimeError("backend not loaded — call load() first")

        image = Image.open(io.BytesIO(image_bytes)).convert("RGB")
        width, height = image.size

        question = _PROMPT_TEMPLATE.format(query=query.strip())
        messages = [{"role": "user", "content": [
            {"type": "image", "image": image},
            {"type": "text", "text": question},
        ]}]

        text = self._processor.py_apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True)
        inputs = self._processor(
            text=[text], images=[image], return_tensors="pt"
        ).to(self.device)

        with torch.inference_mode():
            response = self._model.generate(
                # Must match the weights' dtype, not a hardcoded bfloat16:
                # on a float16 load this would otherwise raise a dtype
                # mismatch in the first vision-tower matmul.
                pixel_values=inputs["pixel_values"].to(self._torch_dtype),
                input_ids=inputs["input_ids"],
                max_new_tokens=self.max_new_tokens,
                generation_mode="hybrid",
            )

        # Decode only the newly generated tokens, keeping special tokens:
        # the <box> markers are special tokens.
        new_tokens = response[:, inputs["input_ids"].shape[1]:]
        answer = self._tokenizer.decode(new_tokens[0], skip_special_tokens=False)

        return parse_boxes(answer, width, height)
