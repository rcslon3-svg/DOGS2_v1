param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot)
)

# This utility is intentionally not part of the firmware build.  It converts
# the OFL-licensed Roboto variable font to compact, four-bit alpha masks.  The
# generated C files are then compiled like any other static firmware asset.
Add-Type -AssemblyName System.Drawing

$fontUrl = 'https://raw.githubusercontent.com/google/fonts/main/ofl/roboto/Roboto%5Bwdth%2Cwght%5D.ttf'
$fontFile = Join-Path $PSScriptRoot 'Roboto.ttf'
if (-not (Test-Path -LiteralPath $fontFile)) {
    Invoke-WebRequest -Uri $fontUrl -OutFile $fontFile -UseBasicParsing
}

$privateFonts = New-Object System.Drawing.Text.PrivateFontCollection
$privateFonts.AddFontFile($fontFile)
$family = $privateFonts.Families[0]

function Convert-Font([string]$prefix, [single]$pointSize) {
    $font = New-Object System.Drawing.Font($family, $pointSize, [System.Drawing.FontStyle]::Regular, [System.Drawing.GraphicsUnit]::Pixel)
    $format = [System.Drawing.StringFormat]::GenericTypographic.Clone()
    $format.FormatFlags = $format.FormatFlags -bor [System.Drawing.StringFormatFlags]::MeasureTrailingSpaces
    $bitmapBytes = New-Object 'System.Collections.Generic.List[byte]'
    $glyphLines = New-Object 'System.Collections.Generic.List[string]'

    for ($code = 32; $code -le 126; $code++) {
        $char = [char]$code
        $bmp = New-Object System.Drawing.Bitmap(64, 64, [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
        $graphics = [System.Drawing.Graphics]::FromImage($bmp)
        $graphics.Clear([System.Drawing.Color]::Transparent)
        $graphics.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::AntiAliasGridFit
        $graphics.DrawString([string]$char, $font, [System.Drawing.Brushes]::White, 4.0, 0.0, $format)
        $advance = [Math]::Max(1, [Math]::Ceiling($graphics.MeasureString([string]$char, $font, 64, $format).Width))

        $minX = 64; $minY = 64; $maxX = -1; $maxY = -1
        for ($y = 0; $y -lt 64; $y++) {
            for ($x = 0; $x -lt 64; $x++) {
                if ($bmp.GetPixel($x, $y).A -ne 0) {
                    if ($x -lt $minX) { $minX = $x }; if ($x -gt $maxX) { $maxX = $x }
                    if ($y -lt $minY) { $minY = $y }; if ($y -gt $maxY) { $maxY = $y }
                }
            }
        }

        $offset = $bitmapBytes.Count
        if ($maxX -lt 0) {
            $width = 0; $height = 0; $xOffset = 0; $yOffset = 0
        } else {
            $width = $maxX - $minX + 1; $height = $maxY - $minY + 1
            $xOffset = $minX - 4; $yOffset = $minY
            $highNibble = $true; $packed = 0
            for ($y = $minY; $y -le $maxY; $y++) {
                for ($x = $minX; $x -le $maxX; $x++) {
                    $alpha = [Math]::Min(15, [Math]::Round($bmp.GetPixel($x, $y).A / 17.0))
                    if ($highNibble) { $packed = $alpha -shl 4; $highNibble = $false }
                    else { $bitmapBytes.Add([byte]($packed -bor $alpha)); $highNibble = $true }
                }
            }
            if (-not $highNibble) { $bitmapBytes.Add([byte]$packed) }
        }
        $glyphLines.Add("    {$offset, $width, $height, $advance, $xOffset, $yOffset}")
        $graphics.Dispose(); $bmp.Dispose()
    }
    $font.Dispose()

    $dataLines = New-Object 'System.Collections.Generic.List[string]'
    for ($i = 0; $i -lt $bitmapBytes.Count; $i += 16) {
        $last = [Math]::Min($i + 15, $bitmapBytes.Count - 1)
        $values = for ($j = $i; $j -le $last; $j++) { '0x{0:X2}' -f $bitmapBytes[$j] }
        $dataLines.Add('    ' + ($values -join ', ') + ',')
    }
    return @{ Prefix=$prefix; Height=[Math]::Ceiling($pointSize * 1.25); Data=$dataLines; Glyphs=$glyphLines }
}

$small = Convert-Font 'roboto_18' 18
$large = Convert-Font 'roboto_30' 30

$header = @"
#pragma once
#include <stdint.h>

typedef struct {
    uint16_t bitmap_offset;
    uint8_t width;
    uint8_t height;
    uint8_t advance;
    int8_t x_offset;
    int8_t y_offset;
} smooth_glyph_t;

typedef struct {
    const uint8_t *bitmap;
    const smooth_glyph_t *glyphs;
    uint8_t first_char;
    uint8_t last_char;
    uint8_t line_height;
} smooth_font_t;

extern const smooth_font_t roboto_18;
extern const smooth_font_t roboto_30;
"@

$source = @"
#include "smooth_font.h"

static const uint8_t roboto_18_bitmap[] = {
$($small.Data -join "`n")
};
static const smooth_glyph_t roboto_18_glyphs[] = {
$($small.Glyphs -join ",`n")
};
const smooth_font_t roboto_18 = {roboto_18_bitmap, roboto_18_glyphs, 32, 126, $($small.Height)};

static const uint8_t roboto_30_bitmap[] = {
$($large.Data -join "`n")
};
static const smooth_glyph_t roboto_30_glyphs[] = {
$($large.Glyphs -join ",`n")
};
const smooth_font_t roboto_30 = {roboto_30_bitmap, roboto_30_glyphs, 32, 126, $($large.Height)};
"@

[IO.File]::WriteAllText((Join-Path $ProjectRoot 'include\smooth_font.h'), $header, [Text.UTF8Encoding]::new($false))
[IO.File]::WriteAllText((Join-Path $ProjectRoot 'src\smooth_font.c'), $source, [Text.UTF8Encoding]::new($false))
Write-Host "Generated Roboto fonts: $($small.Data.Count + $large.Data.Count) data rows"
