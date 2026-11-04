"""Conservative command grammar, independent of ROS and speech SDKs."""
import unicodedata

ALIASES = {
    "出拳": "rock_push", "向前出拳": "rock_push",
    "画八字": "paper_figure8", "画8字": "paper_figure8",
    "画爱心": "heart_draw", "画心": "heart_draw",
    "画圆": "paper_circle", "画个圆": "paper_circle",
}


def normalize(text):
    return "".join(c.lower() for c in unicodedata.normalize("NFKC", text)
                   if not c.isspace() and not unicodedata.category(c).startswith("P"))


def parse_command(text, wake_word=""):
    """Match one complete command with an optional configured wake prefix."""
    text, wake = normalize(text), normalize(wake_word)
    if wake:
        if not text.startswith(wake):
            return None
        text = text[len(wake):]
    if text.startswith("请"):
        text = text[1:]
    return ALIASES.get(text)
