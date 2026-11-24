"""Configure the speaker output and hold its enable GPIO during playback."""

from contextlib import contextmanager
import subprocess


# Physical header pin 37 is GPIO1_B1: bank 1, offset 8 + 1, not GPIO number 37.
ENABLE_CHIP = "gpiochip1"
ENABLE_OFFSET = 9
UNOQ_CARD = "ArduinoImolaHPH"
# JMISC pin 47 is SOC_GPIO_82 and drives VOICE_EN on the expansion board.
UNOQ_ENABLE_OFFSET = 82


def configure_playback(card):
    # Maximum codec playback gain is 0 dB; leave microphone capture controls alone.
    controls = [
        ("Line Playback Switch", "off,off"),
        ("PCM Playback Volume", "127,127"),
        ("Line DAC Playback Volume", "118,118"),
        ("Left Line Mixer DACL1 Switch", "on"),
        ("Right Line Mixer DACR1 Switch", "on"),
        ("Left Line Mixer DACR1 Switch", "off"),
        ("Right Line Mixer DACL1 Switch", "off"),
    ]
    for side in ("Left", "Right"):
        for source in ("PGAL", "PGAR"):
            controls.append(("{} Line Mixer {} Bypass Switch".format(side, source), "off"))
    controls.append(("Line Playback Switch", "on,on"))
    for name, value in controls:
        command = ["amixer", "-c", card, "cset", "name=" + name, value]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                universal_newlines=True, timeout=10)
        if result.returncode:
            raise RuntimeError("{} 失败：{}".format(" ".join(command), result.stdout.strip()))


@contextmanager
def unoq_speaker_enabled(gpiod):
    # The LineOut route follows the board's ALSA UCM profile. Leave gain and MIC2 alone.
    controls = (
        ("RX_CODEC_DMA_RX_0 Audio Mixer MultiMedia2", "1"),
        ("RX_MACRO RX0 MUX", "1"),
        ("RX INT0_1 MIX1 INP0", "RX0"),
        ("RX INT0 DEM MUX", "1"),
        ("LO_RDAC Switch", "1"),
        ("HPHL Switch", "1"),
    )
    for name, value in controls:
        subprocess.run(["amixer", "-c", UNOQ_CARD, "cset", "name=" + name, value],
                       stdout=subprocess.DEVNULL, check=True, timeout=3)
    direction, value = gpiod.line.Direction, gpiod.line.Value
    with gpiod.request_lines("/dev/gpiochip1", consumer="speaker-enable", config={
        UNOQ_ENABLE_OFFSET: gpiod.LineSettings(direction=direction.OUTPUT, output_value=value.ACTIVE),
    }) as lines:
        try:
            yield
        finally:
            lines.set_value(UNOQ_ENABLE_OFFSET, value.INACTIVE)


@contextmanager
def speaker_enabled(verbose=True, device=None):
    try:
        import gpiod
    except ImportError as error:
        raise RuntimeError("缺少 python3-libgpiod；请使用实机系统 Python 运行。") from error

    if device in (f"plughw:{UNOQ_CARD},1", f"hw:{UNOQ_CARD},1"):
        with unoq_speaker_enabled(gpiod):
            yield
        return

    # Keep the request alive throughout playback; releasing a gpioset process
    # immediately after setting high does not guarantee a maintained output.
    with gpiod.Chip(ENABLE_CHIP) as chip:
        line = chip.get_line(ENABLE_OFFSET)
        try:
            line.request(consumer="speaker-enable", type=gpiod.LINE_REQ_DIR_OUT,
                         default_val=1)
        except OSError as error:
            raise RuntimeError("扬声器使能失败：物理引脚 37（GPIO1_B1）：{}".format(
                error)) from error
        try:
            if verbose:
                print("扬声器使能：物理引脚 37（GPIO1_B1）已拉高。", flush=True)
            yield
        finally:
            line.set_value(0)
            if verbose:
                print("扬声器使能：物理引脚 37（GPIO1_B1）已拉低。", flush=True)
