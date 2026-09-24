"""Build printable 1:1 A4 cutting templates from cardboard.scad.

Each part becomes one A4 page of cut outlines (no fill), centred on the page, and all
pages are merged into cardboard/templates.pdf. Print it at 100% / "actual size" and
check the test square with a ruler before cutting.

Usage: python make_templates.py [card thickness in mm, default 2]
"""

import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "cardboard"
PARTS = ["mount", "cap", "upper_arm", "forearm", "hand"]
A4_MM = (210.0, 297.0)
MARGIN_MM = 10.0


def export_sheet(part: str, card_t: str) -> str:
    """Export one part's layer sheet from OpenSCAD and return the SVG text."""
    svg_path = OUT / f"{part}_layers.svg"
    subprocess.run(
        ["openscad", str(HERE / "cardboard.scad"), "-o", str(svg_path),
         "-D", f'sheet="{part}"', "-D", f"card_t={card_t}"],
        check=True, capture_output=True,
    )
    return svg_path.read_text()


def a4_page(svg: str, part: str) -> str:
    """Wrap an OpenSCAD SVG in an A4 page as black 0.3 mm outlines, centred at 1:1."""
    width, height = (float(v) for v in re.search(r'width="([\d.]+)mm" height="([\d.]+)mm"', svg).groups())
    view_box = re.search(r'viewBox="([^"]+)"', svg).group(1)
    body = re.search(r"<svg[^>]*>(.*)</svg>", svg, re.S).group(1)
    body = body.replace('fill="lightgray"', 'fill="none"').replace('stroke-width="0.5"', 'stroke-width="0.3"')
    usable = (A4_MM[0] - 2 * MARGIN_MM, A4_MM[1] - 2 * MARGIN_MM)
    if width > usable[0] or height > usable[1]:
        raise SystemExit(f"{part}: {width:.0f} x {height:.0f} mm does not fit on A4 at 1:1")
    x = (A4_MM[0] - width) / 2
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{A4_MM[0]}mm" height="{A4_MM[1]}mm" '
        f'viewBox="0 0 {A4_MM[0]} {A4_MM[1]}">'
        f'<svg x="{x}" y="{MARGIN_MM}" width="{width}" height="{height}" viewBox="{view_box}">'
        f"{body}</svg></svg>"
    )


def main() -> None:
    card_t = sys.argv[1] if len(sys.argv) > 1 else "2"
    pages = []
    for part in PARTS:
        page_svg = OUT / f"{part}_page.svg"
        page_svg.write_text(a4_page(export_sheet(part, card_t), part))
        page_pdf = OUT / f"{part}_page.pdf"
        subprocess.run(["rsvg-convert", "-f", "pdf", "-o", str(page_pdf), str(page_svg)], check=True)
        pages.append(str(page_pdf))
    subprocess.run(["qpdf", "--empty", "--pages", *pages, "--", str(OUT / "templates.pdf")], check=True)
    print(f"wrote {OUT / 'templates.pdf'} ({len(pages)} A4 pages, card {card_t} mm)")


if __name__ == "__main__":
    main()
