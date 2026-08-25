"""둥근 모서리 조각을 굽는다.

TouchGFX 의 Box 는 사각형뿐이라 모서리를 둥글게 하려면 그 부분만 그림으로
얹어야 한다. 알파는 쓰지 않는다 — 이 화면은 바탕색이 하나로 고정이라
'면 안쪽 / 바탕색 바깥'을 미리 합성해 두면 불투명 이미지로 충분하고,
런타임 블렌딩 비용이 사라진다.

조각에는 곡선 테두리도 같이 굽는다. 온도 칸은 평상시 바탕과 같은 흰 면이라
테두리가 없으면 칸이 어디까지인지 보이지 않는데, 직선 구간은 BoxWithBorder 가
그려 주지만 네 귀는 이 그림이 맡아야 선이 끊기지 않는다.

경계는 4x4 서브샘플로 커버리지를 재서 섞는다. 계단이 보이면 둥근 모서리가
아니라 톱니로 읽힌다.

색을 바꾸면 이 스크립트를 다시 돌려야 한다. 아래 BG/PANELS 값은
Screen1View.cpp 의 색 상수와 같아야 하며, 어긋나면 모서리만 다른 색으로 남는다.

    python tools/make_corner_images.py
"""
import os
import struct
import zlib

R = 10                      # 모서리 반지름 = 조각 한 변
BORDER_W = 1.2              # 테두리 두께. 1.0 이면 안티앨리어싱에 묻혀 흐려진다.
BG = (255, 255, 255)        # #FFFFFF  바탕

# 이름: (면 색, 테두리 색). 물든 칸은 면과 같은 색을 줘서 테두리를 지운다 —
# 면 자체가 이미 경계라 그 위에 선을 더 그으면 칸이 액자처럼 보인다.
PANELS = {
    "white": ((255, 255, 255), (219, 219, 215)),  # #FFFFFF / #DBDBD7
    "warn":  ((194, 118, 26), (194, 118, 26)),    # #C2761A
    "crit":  ((179, 63, 56), (179, 63, 56)),      # #B33F38
}
# 모서리별 원 중심. 조각 안에서 '살이 붙어 있는' 쪽 반대편 코너가 중심이다.
CENTERS = {"tl": (R, R), "tr": (0, R), "bl": (R, 0), "br": (0, 0)}

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "TouchGFX", "assets", "images")


def coverage(px, py, cx, cy, radius):
    """픽셀 하나가 반지름 radius 원 안에 얼마나 들어와 있는지 0..1 로 돌려준다."""
    hit = 0
    for sy in range(4):
        for sx in range(4):
            x = px + (sx + 0.5) / 4.0
            y = py + (sy + 0.5) / 4.0
            if ((x - cx) ** 2 + (y - cy) ** 2) <= (radius * radius):
                hit += 1
    return hit / 16.0


def write_png(path, width, height, rows):
    raw = b"".join(b"\x00" + bytes(r) for r in rows)

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    blob = (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", header)
            + chunk(b"IDAT", zlib.compress(raw, 9))
            + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(blob)


def main():
    out = os.path.normpath(OUT)
    os.makedirs(out, exist_ok=True)
    made = []

    for name, (panel, border) in PANELS.items():
        for corner, (cx, cy) in CENTERS.items():
            rows = []
            for y in range(R):
                row = []
                for x in range(R):
                    # 바깥 원까지가 칸, 안쪽 원까지가 면, 그 사이가 테두리다.
                    outer = coverage(x, y, cx, cy, R)
                    inner = coverage(x, y, cx, cy, R - BORDER_W)
                    ring = outer - inner
                    for i in range(3):
                        v = (panel[i] * inner
                             + border[i] * ring
                             + BG[i] * (1.0 - outer))
                        row.append(int(round(v)))
                rows.append(row)
            path = os.path.join(out, "corner_%s_%s.png" % (name, corner))
            write_png(path, R, R, rows)
            made.append((os.path.basename(path), os.path.getsize(path)))

    for n, s in made:
        print("%-24s %d bytes" % (n, s))


if __name__ == "__main__":
    main()
