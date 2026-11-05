"""Conservative command grammar, independent of ROS and speech SDKs."""
import unicodedata
import re

# Normalize vocabulary used by commands and rejection rules, without an extra
# runtime dependency. Whisper can emit Traditional Chinese even with language=zh.
TRADITIONAL = str.maketrans({
    '畫': '画', '個': '个', '圓': '圆', '愛': '爱', '劃': '划',
    '請': '请', '幫': '帮', '給': '给', '機': '机', '臂': '臂',
    '謝': '谢', '嗎': '吗', '揮': '挥', '園': '园', '來': '来',
    '別': '别', '後': '后', '還': '还', '說': '说', '聽': '听',
    '為': '为', '麼': '么', '們': '们', '煩': '烦', '這': '这',
})

ALIASES = {
    "出拳": "rock_push", "向前出拳": "rock_push",
    "画八字": "paper_figure8", "画8字": "paper_figure8",
    "画爱心": "heart_draw", "画心": "heart_draw",
    "画圆": "paper_circle", "画个圆": "paper_circle",
}

# Deliberately bounded fuzzy matching: known wording/ASR substitutions only.
# Never select the nearest action for arbitrary conversation.
PREFIX = re.compile(r'^(?:你好|机械臂|小助手|请你|请|麻烦你|麻烦|能不能|可以|帮我|给我|来|再|一下)*(?:给大家|给我们)?')
SUFFIX = re.compile(r'(?:一下|一个|一次|吧|呀|啊|好吗|可以吗|谢谢)+$')
BLOCKED = ('不', '别', '停', '取消', '禁止', '然后', '再来', '或者', '还是',
           '如果', '要是', '说', '听', '知道', '为什么', '怎么', '能否')


def natural_command(text):
    text = text.replace('能不能', '可以')
    if any(word in text for word in BLOCKED):
        return None
    text = PREFIX.sub('', text)
    text = SUFFIX.sub('', text)
    patterns = {
        'rock_push': r'(?:出(?:个|一)?拳|打(?:个|一)?拳|向前出拳|挥(?:个|一)?拳)',
        'paper_figure8': r'(?:画|划)(?:一个|个)?(?:八|8)(?:字|字符)?',
        'heart_draw': r'(?:画|划)(?:一个|个)?(?:爱心|心形|心|爱星)',
        'paper_circle': r'(?:画|划)(?:一个|个)?(?:圆|圆圈|圆形|圈|园)',
    }
    matches = [track for track, pattern in patterns.items()
               if re.fullmatch(pattern, text)]
    return matches[0] if len(matches) == 1 else None


def normalize(text):
    return "".join(c.lower() for c in unicodedata.normalize("NFKC", text).translate(TRADITIONAL)
                   if not c.isspace() and not unicodedata.category(c).startswith("P"))


def parse_command(text, wake_word="", fuzzy=True):
    """Match one complete command with an optional configured wake prefix."""
    text, wake = normalize(text), normalize(wake_word)
    if wake:
        if not text.startswith(wake):
            return None
        text = text[len(wake):]
    if fuzzy:
        return ALIASES.get(text) or natural_command(text)
    if text.startswith("请"):
        text = text[1:]
    return ALIASES.get(text)
