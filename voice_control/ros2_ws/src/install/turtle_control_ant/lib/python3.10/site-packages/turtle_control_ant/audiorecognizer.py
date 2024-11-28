import torch
import torchaudio
from speechbrain.pretrained import EncoderClassifier


class Extract_embeddings:
    def __init__(self, threshold=0.5, data_patch_audio=None):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.recognizer = self._initialize_recognizer()
        self.data_audio = {}
        self.id = 0
        self.threshold = threshold
        self.data_patch_audio = data_patch_audio

    def _initialize_recognizer(self):
        return EncoderClassifier.from_hparams(
            source="speechbrain/spkrec-ecapa-voxceleb",
            savedir="../tmp_model",
            run_opts={"device": self.device},
        )

    def _process_audio_file(self, audio):
        try:
            with torch.no_grad():
                signal, fs = torchaudio.load(audio)
                if fs != 16000:
                    resampler = torchaudio.transforms.Resample(
                        orig_freq=fs,
                        new_freq=16000,
                    )
                    signal = resampler(signal)
                embedding = self.recognizer.encode_batch(signal.to(self.device))
                return embedding.squeeze(0).cpu().numpy()
        except Exception as e:
            raise ValueError(f"Error processing audio {audio}")

    def collect_embedding(self, file_path):
        embedding = self._process_audio_file(file_path)
        if embedding is None:
            raise ValueError("Failed to collect embedding.")
        return embedding