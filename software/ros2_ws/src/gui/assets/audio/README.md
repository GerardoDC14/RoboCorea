# Vosk speech model

Place a Vosk model's contents **directly in this directory** so the layout is:

```
gui/assets/audio/
  am/
  conf/
  graph/
  ivector/
  README
```

Download a model from <https://alphacephei.com/vosk/models>. The small English
model (`vosk-model-small-en-us-0.15`) is recommended for low latency. Extract it
and move the inner folders here (Vosk is given this directory path, not a file).

If the model is absent the GUI still runs — speech transcription is simply
disabled and the dashboard logs a warning.

The Vosk **library** itself (`libvosk.so` + `vosk_api.h`) is installed under
`/usr/local` (see the legacy `shared/gui_ws/README.md` §2 for the one-time
install commands).
