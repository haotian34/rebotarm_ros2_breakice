from glob import glob
from setuptools import setup

setup(
    name="rebotarm_voice", version="0.1.0", packages=["rebotarm_voice"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/rebotarm_voice"]),
        ("share/rebotarm_voice", ["package.xml", "README.md"]),
        ("share/rebotarm_voice/launch", glob("launch/*.py")),
        ("share/rebotarm_voice/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "faster-whisper>=1.1,<2", "numpy", "sounddevice>=0.4"],
    maintainer="reBotArm Maintainers", maintainer_email="support@example.com",
    description="Whisper speech recognition and named trajectory control",
    license="MIT",
    entry_points={"console_scripts": [
        "rviz_subtitles = rebotarm_voice.rviz_subtitles:main",
        "download_model = rebotarm_voice.audio:download_model",
        "voice_control_node = rebotarm_voice.voice_control_node:main",
    ]},
)
