#!/usr/bin/env python3
"""Extract text from all 5 PPTX chapter files."""
import sys
import zipfile
from pathlib import Path

NS = {
    "a": "http://schemas.openxmlformats.org/drawingml/2006/main",
    "p": "http://schemas.openxmlformats.org/presentationml/2006/main",
    "r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
    "pkg": "http://schemas.openxmlformats.org/package/2006/relationships",
}

try:
    from lxml import etree
    def _parse(blob):
        return etree.fromstring(blob)
    def _findall(node, xpath):
        return node.xpath(xpath, namespaces=NS)
    def _findone(node, xpath):
        result = node.xpath(xpath, namespaces=NS)
        return result[0] if result else None
    def _tag(node):
        raw = node.tag
        for prefix, uri in NS.items():
            marker = "{" + uri + "}"
            if raw.startswith(marker):
                return f"{prefix}:{raw[len(marker):]}"
        return raw
except ImportError:
    from xml.etree import ElementTree as etree
    def _parse(blob):
        return etree.fromstring(blob)
    def _findall(node, xpath):
        return node.findall(xpath, NS)
    def _findone(node, xpath):
        return node.find(xpath, NS)
    def _tag(node):
        raw = node.tag
        for prefix, uri in NS.items():
            marker = "{" + uri + "}"
            if raw.startswith(marker):
                return f"{prefix}:{raw[len(marker):]}"
        return raw


def _resolve(base, target):
    if target.startswith("/"):
        return target.lstrip("/")
    stack = []
    combined = f"{base}/{target}" if base else target
    for segment in combined.split("/"):
        if segment == "..":
            if stack:
                stack.pop()
        elif segment and segment != ".":
            stack.append(segment)
    return "/".join(stack)


def _read_rels(zf, rels_path):
    if rels_path not in zf.namelist():
        return {}
    root = _parse(zf.read(rels_path))
    base = str(Path(rels_path).parent.parent.as_posix())
    base = base if base != "." else ""
    resolved = {}
    for rel in _findall(root, "./pkg:Relationship"):
        rid = rel.get("Id")
        target = rel.get("Target") or ""
        mode = (rel.get("TargetMode") or "Internal").lower()
        if rid is None or mode != "internal":
            continue
        resolved[rid] = _resolve(base, target)
    return resolved


def _ordered_slide_parts(zf):
    presentation_path = "ppt/presentation.xml"
    presentation_rels = "ppt/_rels/presentation.xml.rels"
    if presentation_path not in zf.namelist():
        return sorted(name for name in zf.namelist()
                      if name.startswith("ppt/slides/slide") and name.endswith(".xml"))
    presentation = _parse(zf.read(presentation_path))
    rels = _read_rels(zf, presentation_rels)
    order = []
    for sld_id in _findall(presentation, ".//p:sldIdLst/p:sldId"):
        rid = sld_id.get("{" + NS["r"] + "}id") or sld_id.get("r:id")
        if rid is None:
            continue
        target = rels.get(rid)
        if target and target in zf.namelist():
            order.append(target)
    return order


def _notes_for(zf, slide_part):
    slide_name = Path(slide_part).name
    rels_path = f"ppt/slides/_rels/{slide_name}.rels"
    for target in _read_rels(zf, rels_path).values():
        if "notesSlides/" in target and target.endswith(".xml") and target in zf.namelist():
            return target
    return None


def _run_text(run):
    return "".join(t.text for t in _findall(run, "./a:t") if t.text)


def _paragraph_text(paragraph):
    parts = []
    for child in paragraph:
        name = _tag(child)
        if name == "a:r":
            parts.append(_run_text(child))
        elif name == "a:br":
            parts.append("\n")
        elif name == "a:fld":
            parts.append(_run_text(child))
    return "".join(parts)


def _slide_title(slide):
    for sp in _findall(slide, ".//p:sp"):
        for ph in _findall(sp, ".//p:nvSpPr/p:nvPr/p:ph"):
            if ph.get("type") in {"title", "ctrTitle"}:
                text = "".join(_paragraph_text(p) for p in _findall(sp, ".//a:p"))
                if text.strip():
                    return text.strip()
    return ""


def _paragraphs_with_bullet_flag(slide, skip_title=False):
    for sp in _findall(slide, ".//p:sp"):
        if skip_title:
            is_title = False
            for ph in _findall(sp, ".//p:nvSpPr/p:nvPr/p:ph"):
                if ph.get("type") in {"title", "ctrTitle"}:
                    is_title = True
                    break
            if is_title:
                continue
        for paragraph in _findall(sp, ".//a:p"):
            text = _paragraph_text(paragraph).strip()
            if not text:
                continue
            pPr = _findone(paragraph, "./a:pPr")
            has_bullet = False
            level = 0
            if pPr is not None:
                try:
                    level = int(pPr.get("lvl") or "0")
                except ValueError:
                    level = 0
                if _findone(pPr, "./a:buNone") is None:
                    for child in pPr:
                        if _tag(child).startswith("a:bu"):
                            has_bullet = True
                            break
            yield text, has_bullet, level


def _slide_tables_markdown(slide):
    for tbl in _findall(slide, ".//a:tbl"):
        rows = []
        for tr in _findall(tbl, ".//a:tr"):
            row = []
            for tc in _findall(tr, ".//a:tc"):
                cell = " ".join(_paragraph_text(p) for p in _findall(tc, ".//a:p")).strip()
                row.append(cell.replace("|", "\\|") or " ")
            rows.append(row)
        if not rows:
            continue
        width = max(len(r) for r in rows)
        rows = [r + [" "] * (width - len(r)) for r in rows]
        yield ""
        yield "| " + " | ".join(rows[0]) + " |"
        yield "| " + " | ".join("---" for _ in range(width)) + " |"
        for r in rows[1:]:
            yield "| " + " | ".join(r) + " |"


def _notes_lines(zf, notes_part):
    try:
        notes = _parse(zf.read(notes_part))
    except Exception:
        return []
    out = []
    for sp in _findall(notes, ".//p:sp"):
        skip = False
        for ph in _findall(sp, ".//p:nvSpPr/p:nvPr/p:ph"):
            if ph.get("type") in {"sldImg", "sldNum"}:
                skip = True
                break
        if skip:
            continue
        for paragraph in _findall(sp, ".//a:p"):
            text = _paragraph_text(paragraph).strip()
            if text:
                out.append(text)
    return out


def dump(path, include_notes=True, include_tables=True):
    chunks = []
    with zipfile.ZipFile(path) as zf:
        for i, slide_part in enumerate(_ordered_slide_parts(zf), start=1):
            try:
                slide = _parse(zf.read(slide_part))
            except Exception as exc:
                print(f"warning: {slide_part}: {exc}", file=sys.stderr)
                continue

            title = _slide_title(slide)
            heading = f"## Slide {i}" + (f": {title}" if title else "")
            out = [heading, ""]
            for text, bullet, level in _paragraphs_with_bullet_flag(slide, skip_title=True):
                if bullet:
                    out.append("  " * level + "- " + text)
                else:
                    out.append(text)
            out.extend(_slide_tables_markdown(slide))

            if include_notes:
                notes_part = _notes_for(zf, slide_part)
                if notes_part:
                    notes = _notes_lines(zf, notes_part)
                    if notes:
                        out.append("")
                        out.append("> **Speaker notes**")
                        out.extend(f"> {ln}" for ln in notes)

            if len(out) > 2:
                chunks.extend(out)
                chunks.append("")

    return "\n".join(chunks).rstrip() + "\n"


files = [
    ("第9章 四足机器人结构设计与安装流程.pptx", "Chapter 9"),
    ("第10章 四足机器人硬件和电路结构介绍.pptx", "Chapter 10"),
    ("第11章 基于模型预测控制的四足机器人全身运动控制方法.pptx", "Chapter 11"),
    ("第12章 四足机器人运动控制程序框架介绍.pptx", "Chapter 12"),
    ("第13章  四足机器人实验仿真与验证.pptx", "Chapter 13"),
]

base = "/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/四足仿生机器人基本原理及开发教程（电子版课件）"

for fname, label in files:
    fpath = Path(base) / fname
    if not fpath.exists():
        print(f"FILE NOT FOUND: {fpath}", file=sys.stderr)
        continue
    print(f"===== {label}: {fname} =====")
    print(f"File size: {fpath.stat().st_size} bytes")
    text = dump(fpath, include_notes=True, include_tables=True)
    print(text)
    print()
