#!/usr/bin/env python3
"""Render docs/*.md through the Jekyll layout so the site can be checked in a
browser without a Ruby toolchain. Writes docs/_preview_<name>.html, which is
gitignored - it exists to be looked at, not published.

Usage:  python3 tools/preview_docs.py   (from the repo root)
"""
import re,markdown,pathlib,sys
lay=pathlib.Path('docs/_layouts/default.html').read_text()
for name in ('the-shift','capability-map'):
    src=pathlib.Path(f'docs/{name}.md').read_text()
    fm,body=re.match(r'^---\n(.*?)\n---\n(.*)$', src, re.S).groups()
    meta=dict(re.findall(r'^(\w+):\s*"?(.*?)"?$', fm, re.M))
    html=markdown.markdown(body, extensions=['tables','fenced_code','toc'])
    html=html.replace('<table>','<div class="tw"><table>').replace('</table>','</table></div>')
    out=lay.replace('{{ content }}',html)
    out=re.sub(r"\{\{ page\.title \}\}", meta.get('title',''), out)
    out=re.sub(r"\{\{ page\.description \}\}", meta.get('description',''), out)
    out=re.sub(r"\{\{ '([^']+)' \| relative_url \}\}", lambda m: '.'+m.group(1), out)
    pathlib.Path(f'docs/_preview_{name}.html').write_text(out)
print('previews rebuilt')
