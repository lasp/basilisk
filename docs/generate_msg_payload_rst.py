"""Generate RST files for message payload definitions.

Scans src/architecture/msgPayloadDef/*.h and generates one RST per header
with a lowercase label and a doxygenstruct directive, plus an index.rst.
"""

import os
import re
from pathlib import Path


def extract_struct_name(header_path):
    """Extract the typedef struct name from a payload header file."""
    text = Path(header_path).read_text(errors="ignore")
    # Match: } SomePayloadName;
    m = re.search(r"\}\s*(\w+(?:Payload|MsgPayload))\s*;", text)
    if m:
        return m.group(1)
    # Fallback: use filename
    return Path(header_path).stem


def generate_payload_rst(header_path, output_dir):
    """Generate a single RST file for a message payload header."""
    struct_name = extract_struct_name(header_path)
    # Ignore headers that do not define a Payload struct
    if not struct_name.endswith("Payload"):
        return None, struct_name

    label = struct_name.lower()
    rst_filename = f"{struct_name}.rst"
    underline = "=" * len(struct_name)

    content = f""".. _{label}:

{struct_name}
{underline}

.. doxygenstruct:: {struct_name}
   :members:
"""
    output_path = os.path.join(output_dir, rst_filename)
    with open(output_path, "w") as f:
        f.write(content)
    return rst_filename, struct_name


def generate_index(rst_filenames, output_dir):
    """Generate index.rst for all payload RST files."""
    entries = sorted(os.path.splitext(f)[0] for f in rst_filenames)
    lines = [
        ".. _msgpayloads:",
        "",
        "Message Payloads",
        "================",
        "",
        ".. toctree::",
        "   :maxdepth: 1",
        "   :hidden:",
        "",
    ]
    lines.extend(f"   {e}" for e in entries)
    lines.append("")
    # Visible list of links (toctree is hidden for nav-tree only)
    lines.append("")
    for e in entries:
        lines.append(f"- :doc:`{e}`")
    lines.append("")

    with open(os.path.join(output_dir, "index.rst"), "w") as f:
        f.write("\n".join(lines))


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    payload_dir = os.path.join(script_dir, "..", "src", "architecture", "msgPayloadDef")
    output_dir = os.path.join(script_dir, "source", "api", "msgPayloads")

    os.makedirs(output_dir, exist_ok=True)

    headers = sorted(Path(payload_dir).glob("*.h"))
    if not headers:
        print(f"No headers found in {payload_dir}")
        return

    rst_filenames = []
    for h in headers:
        rst_filename, struct_name = generate_payload_rst(str(h), output_dir)
        if rst_filename is None:
            print(f"  Skipped {h.name} (no MsgPayload struct)")
            continue
        rst_filenames.append(rst_filename)
        print(f"  Generated {rst_filename} for {struct_name}")

    generate_index(rst_filenames, output_dir)
    print(f"Generated {len(rst_filenames)} payload RSTs + index in {output_dir}")


if __name__ == "__main__":
    main()
