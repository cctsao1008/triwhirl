# Active upright identification notes

The first actively excited near-upright BLE run (`body-active-01`) is retained as diagnostic evidence, not as controller-synthesis plant evidence.

Two experiment-design problems were exposed:

1. The selected contact posture was not held constant. Passive local identification established the chosen upright vertex at about 67–69 deg in the current IMU frame, but active trials 1/2/4 were around -55 to -57 deg (a neighboring Reuleaux vertex, about 120 deg away) and trial 3 was about +2.8 deg. Different contacts must not be mixed into one local linear plant.
2. The host waited for release detection before sending `motor vq`. Measured firmware telemetry shows first nonzero Vq only about 62–122 ms after the release trigger. The previously identified open-loop unstable e-folding time is about 56 ms, so the excitation arrived too late relative to the body divergence. Trial 4 yielded only one measured nonzero-Vq telemetry row.

The active acquisition flow is therefore changed to:

- require the selected upright vertex near 68 deg (default tolerance ±12 deg) and reject/retry other contact postures;
- command signed Vq while the user still holds the body;
- wait until firmware telemetry confirms the requested Vq is established and the held posture remains near the reference;
- only then instruct the user to release without pushing;
- measure post-release dynamics using telemetry `vq_v` as the authoritative input;
- keep both positive and negative Vq coverage and reject fits with insufficient active samples per sign.

This sequencing removes host/BLE command latency from the post-release input timing while excluding held rows from the plant regression.
