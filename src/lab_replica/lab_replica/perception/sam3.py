"""SAM 3 (Segment Anything with concepts): text prompt in, instance masks out.

The image is encoded once and every prompt is decoded against the same
features, so a dozen item names cost about one image encoding plus a dozen
light decoder passes. Needs torch and transformers >= 5 (in the workspace's
.venv) and the gated checkpoint facebook/sam3: HF_TOKEN in the environment or
in <workspace>/.env, or the checkpoint already in the Hugging Face cache.
"""

import os
from dataclasses import dataclass

import numpy as np

MODEL_ID = "facebook/sam3"


@dataclass
class Instance:
    prompt: str
    score: float
    mask: np.ndarray      # (H, W) bool
    box: tuple            # (x0, y0, x1, y1) pixels


def load_dotenv(path):
    """Set variables from a KEY=VALUE file that are not set yet."""
    if not os.path.isfile(path):
        return
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


class Sam3:
    def __init__(self, model_id=MODEL_ID, device=None):
        import torch
        from transformers import Sam3Model, Sam3Processor

        from .. import config
        load_dotenv(os.path.join(config.workspace_root(), ".env"))
        self.torch = torch
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.dtype = torch.bfloat16 if self.device.startswith("cuda") else torch.float32
        self.processor = Sam3Processor.from_pretrained(model_id)
        self.model = Sam3Model.from_pretrained(model_id, dtype=self.dtype).to(self.device).eval()
        self._text_cache = {}

    def _text(self, prompt):
        if prompt not in self._text_cache:
            t = self.processor(text=prompt, return_tensors="pt")
            self._text_cache[prompt] = (t["input_ids"].to(self.device), t["attention_mask"].to(self.device))
        return self._text_cache[prompt]

    def segment(self, image, prompts, threshold=0.3, mask_threshold=0.5):
        """{prompt: [Instance, ...] sorted by score} for an RGB image (H, W, 3 uint8)."""
        from PIL import Image
        torch = self.torch
        img = Image.fromarray(np.asarray(image))
        inputs = self.processor(images=img, return_tensors="pt")
        size = [img.size[::-1]]
        out = {}
        with torch.inference_mode():
            pv = inputs["pixel_values"].to(self.device, self.dtype)
            vision = self.model.get_vision_features(pixel_values=pv)
            for prompt in prompts:
                ids, att = self._text(prompt)
                res = self.model(vision_embeds=vision, input_ids=ids, attention_mask=att)
                post = self.processor.post_process_instance_segmentation(
                    res, threshold=threshold, mask_threshold=mask_threshold, target_sizes=size)[0]
                inst = [Instance(prompt, float(s), m.cpu().numpy().astype(bool), tuple(float(v) for v in b))
                        for s, m, b in zip(post["scores"].float(), post["masks"], post["boxes"].float())]
                out[prompt] = sorted(inst, key=lambda i: -i.score)
        return out
