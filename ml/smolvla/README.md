# SmolVLA action contract

The fine-tuned policy action vector has five elements. The largest element selects one skill:

| Index | Skill |
| --- | --- |
| 0 | `go_to_viewpoint` |
| 1 | `rotate_scan` |
| 2 | `approach_target` |
| 3 | `report_not_found` |
| 4 | `stop` |

The policy is advisory. The ROS coordinator must retain collision checking, timeout handling and a deterministic fallback. Model weights and datasets are not committed to Git.
