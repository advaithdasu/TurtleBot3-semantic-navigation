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


class LocateAnythingBackend:

    def __init__(self, model_id: str = _MODEL_ID, device: str = "cuda",
                 max_new_tokens: int = 2048, **_kwargs) -> None:
        self.model_id = model_id
        self.device = device
        self.max_new_tokens = max_new_tokens
        self._model = None
        self._tokenizer = None
        self._processor = None

    def load(self) -> None:
        import torch
        from transformers import AutoModel, AutoProcessor, AutoTokenizer

        t0 = time.monotonic()
        self._tokenizer = AutoTokenizer.from_pretrained(
            self.model_id, trust_remote_code=True)
        self._processor = AutoProcessor.from_pretrained(
            self.model_id, trust_remote_code=True)
        self._model = (
            AutoModel.from_pretrained(
                self.model_id,
                torch_dtype=torch.bfloat16,
                trust_remote_code=True,
            )
            .to(self.device)
            .eval()
        )
        print(f"[locate_anything] loaded {self.model_id} on {self.device} "
              f"in {time.monotonic() - t0:.1f}s")

    def info(self) -> dict:
        return {
            "backend": "locate_anything",
            "device": self.device,
            "model": self.model_id,
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
                pixel_values=inputs["pixel_values"].to(torch.bfloat16),
                input_ids=inputs["input_ids"],
                max_new_tokens=self.max_new_tokens,
                generation_mode="hybrid",
            )

        # Decode only the newly generated tokens, keeping special tokens:
        # the <box> markers are special tokens.
        new_tokens = response[:, inputs["input_ids"].shape[1]:]
        answer = self._tokenizer.decode(new_tokens[0], skip_special_tokens=False)

        return parse_boxes(answer, width, height)
