from pathlib import Path

from PIL import Image


WIDTH = 320
HEIGHT = 170


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    source = root / "dogs2-pro-all-you-need-transparent.png"
    preview = root / "splash_preview.png"
    output = root / "src" / "splash_bitmap.c"

    image = Image.open(source).convert("RGBA")
    scale = WIDTH / image.width
    resized = image.resize(
        (WIDTH, round(image.height * scale)),
        Image.Resampling.LANCZOS,
    )

    canvas = Image.new("RGBA", (WIDTH, HEIGHT), (0, 0, 0, 255))
    canvas.alpha_composite(resized, (0, (HEIGHT - resized.height) // 2))
    rgb = canvas.convert("RGB")
    rgb.save(preview)

    data = bytearray()
    for red, green, blue in rgb.getdata():
        pixel = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3)
        data.append((pixel >> 8) & 0xFF)
        data.append(pixel & 0xFF)

    with output.open("w", encoding="ascii", newline="\n") as file:
        file.write('#include "splash_bitmap.h"\n\n')
        file.write("const size_t splash_bitmap_rgb565_size = sizeof(splash_bitmap_rgb565);\n\n")
        file.write(
            "const uint8_t splash_bitmap_rgb565"
            "[SPLASH_BITMAP_WIDTH * SPLASH_BITMAP_HEIGHT * 2U] = {\n"
        )
        for offset in range(0, len(data), 16):
            chunk = data[offset : offset + 16]
            file.write("    ")
            file.write(", ".join(f"0x{value:02X}" for value in chunk))
            file.write(", \n")
        file.write("};\n")

    print(
        f"{source.name} {image.size} -> {output.relative_to(root)} "
        f"{WIDTH}x{HEIGHT}, {len(data)} bytes"
    )


if __name__ == "__main__":
    main()
