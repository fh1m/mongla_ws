# Hand-labelled truth for the studies: when a target is really in view, labelled by eye from the frames and independently of any model, so a tracker or detector can be scored rather than compared with another.

One JSON per clip, read by `tracker_ab.py --truth` (`score_truth` documents
the format). A label names the clip exactly, because two archive clips share
the name `gate.mkv` (measured-bars section 88).
