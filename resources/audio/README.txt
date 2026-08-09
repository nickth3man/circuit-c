Audio Assets for Circuit
=======================

Drop the following three WAV files into this directory to activate the
sample-based audio system. Until they are present, the game runs silently
(no errors are printed).

  1. engine.wav   — short looping engine tone (~1 second, mono recommended).
                    Pitch is modulated from 0.5× (idle) to 2.0× (redline).

  2. screech.wav  — short looping tire-screech (~0.5 second, mono recommended).
                    Volume fades in with slide intensity and speed (0–30 m/s).

  3. thud.wav     — short one-shot impact sound (~0.2–0.3 second).
                    Played on significant barrier collisions with a 0.1 s
                    rate limit to avoid machine-gunning rapid contacts.

Format: 16-bit PCM WAV at 44100 Hz is the safest choice for raylib's audio
backend. Other sample rates and formats may work but are not tested.

The game references these by relative path: resources/audio/<name>.wav
Run circuit.exe from the repository root.
