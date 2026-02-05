from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]  # repo root (tools/의 부모)
OUT = ROOT / "docs" / "FILE_INDEX.md"

EXCLUDE_DIRS = {"build", "install", "log", ".git", "__pycache__"}

def is_excluded(p: Path) -> bool:
    return any(part in EXCLUDE_DIRS for part in p.parts)

def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)

    files = []
    for p in ROOT.rglob("*"):
        if p.is_dir():
            continue
        if is_excluded(p):
            continue
        files.append(p)

    def rel(p): return p.relative_to(ROOT).as_posix()

    with OUT.open("w", encoding="utf-8") as f:
        f.write("# Repo File Index\n\n")
        f.write("자동 생성: tools/repo_inventory.py\n\n")
        f.write("## Packages (src)\n\n")

        src = ROOT / "src"
        if src.exists():
            pkgs = sorted([d for d in src.iterdir() if d.is_dir() and not d.name.startswith(".")])
            for pkg in pkgs:
                f.write(f"### {pkg.name}\n\n")
                for fp in sorted([x for x in pkg.rglob('*') if x.is_file() and not is_excluded(x)]):
                    f.write(f"- `{rel(fp)}`\n")
                f.write("\n")
        else:
            f.write("- (no src directory found)\n")

        f.write("\n## Other files\n\n")
        for fp in sorted(files):
            if "src/" in rel(fp):
                continue
            f.write(f"- `{rel(fp)}`\n")

    print(f"[OK] wrote {OUT}")

if __name__ == "__main__":
    main()
