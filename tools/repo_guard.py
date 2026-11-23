#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仓库护栏：检查必备文件、README 外观层与体积。

用法（仓库根目录）：
    python3 repo_guard.py            # 硬性项不过 → 退出码 1；体积/徽章只告警
    python3 repo_guard.py --strict   # 体积超限也算失败
    python3 repo_guard.py --summary  # 额外写 GitHub Step Summary

口径见 xhtfps/REPO-STANDARD.md（§2 必备文件、§3 五段、§3.1 外观层、§2.2 体积红线）。
"""

import argparse
import os
import re
import subprocess
import sys
import urllib.request

REQUIRED_FILES = [
    "README.md",
    "LICENSE",
    ".gitignore",
    ".gitattributes",
    "CONTRIBUTING.md",
    "SECURITY.md",
    ".github/ISSUE_TEMPLATE/bug_report.yml",
    ".github/ISSUE_TEMPLATE/feature_request.yml",
    ".github/ISSUE_TEMPLATE/config.yml",
    ".github/PULL_REQUEST_TEMPLATE.md",
]

MAX_FILE_MB = 5
MAX_REPO_MB = 50
FAIL = "FAIL"
WARN = "WARN"
OK = "OK"

def tracked_files():
    out = subprocess.run(
        ["git", "ls-tree", "-r", "-l", "--full-tree", "HEAD"],
        capture_output=True, text=True, check=True,
    ).stdout
    rows = []
    for line in out.splitlines():
        meta, path = line.split("\t", 1)
        parts = meta.split()
        size = int(parts[3]) if parts[3].isdigit() else 0
        rows.append((path, size))
    return rows

def head_text(path):
    try:
        return subprocess.run(
            ["git", "show", f"HEAD:{path}"], capture_output=True, text=True, check=True
        ).stdout
    except subprocess.CalledProcessError:
        return ""

def check_structure(readme):
    issues, warns = [], []
    numbered = sorted({int(m.group(1)) for m in re.finditer(r"(?m)^##\s*(\d+)[\.\s]", readme)})
    missing = [n for n in (1, 2, 3, 4, 5) if n not in numbered]
    if missing:
        issues.append(f"README 缺少编号段：{missing}（应有 §1–§5）")
    if "```mermaid" not in readme:
        issues.append("README 缺少 ```mermaid 结构图")
    if not re.search(r"(?m)^##\s*路线图", readme):
        issues.append("README 缺少「## 路线图」")
    badges = re.findall(r"!\[[^\]]*\]\((https://img\.shields\.io/[^)]+)\)", readme)
    if len(badges) < 3:
        issues.append(f"README 徽章不足 3 个（实际 {len(badges)}）")
    if "CONTRIBUTING.md" not in readme:
        warns.append("README 未链接 CONTRIBUTING.md")
    if "SECURITY.md" not in readme:
        warns.append("README 未链接 SECURITY.md")
    return issues, warns, badges

def check_badges(badges):
    bad = []
    for url in badges:
        for _ in range(3):
            try:
                req = urllib.request.Request(url, method="HEAD", headers={"User-Agent": "repo-guard"})
                with urllib.request.urlopen(req, timeout=10) as resp:
                    if resp.status < 400:
                        break
            except Exception:
                continue
        else:
            bad.append(url)
    return bad

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--strict", action="store_true", help="体积超限也判失败")
    ap.add_argument("--summary", action="store_true", help="写 GitHub Step Summary")
    ap.add_argument("--skip-badges", action="store_true", help="跳过徽章联网检查")
    args = ap.parse_args()

    files = tracked_files()
    names = {p for p, _ in files}
    total_mb = sum(s for _, s in files) / 1024 / 1024
    issues, warns = [], []

    for f in REQUIRED_FILES:
        if f not in names:
            issues.append(f"缺少必备文件：{f}")

    readme = head_text("README.md")
    if not readme:
        issues.append("README.md 为空或不可读")
        badges = []
    else:
        s_issues, s_warns, badges = check_structure(readme)
        issues += s_issues
        warns += s_warns

    big = [(p, s / 1024 / 1024) for p, s in files if s > MAX_FILE_MB * 1024 * 1024]
    for p, mb in big:
        msg = f"单文件超 {MAX_FILE_MB} MB：{p}（{mb:.1f} MB）"
        (issues if args.strict else warns).append(msg)
    if total_mb > MAX_REPO_MB:
        msg = f"单仓体积 {total_mb:.1f} MB 超红线 {MAX_REPO_MB} MB"
        (issues if args.strict else warns).append(msg)

    if badges and not args.skip_badges:
        bad = check_badges(badges)
        for u in bad:
            warns.append(f"徽章无法访问：{u}")

    lines = ["## 仓库护栏检查", "", f"- 仓库体积：{total_mb:.1f} MB / 单文件上限 {MAX_FILE_MB} MB"]
    lines += [f"- ❌ {i}" for i in issues] + [f"- ⚠️ {w}" for w in warns]
    if not issues:
        lines.append("- ✅ 硬性项全部通过")
    text = "\n".join(lines)
    print(text)
    if args.summary and os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a", encoding="utf-8") as fh:
            fh.write(text + "\n")

    return 1 if issues else 0

if __name__ == "__main__":
    sys.exit(main())
