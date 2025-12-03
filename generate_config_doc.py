#!/bin/env python3

import re
import subprocess
import tree_sitter_rust as tsrust
from tree_sitter import Language, Parser
import os
from typing import List, Tuple
import toml

import IPython

# Read version from root Cargo.toml (workspace)
root_cargo_toml_path = "Cargo.toml"
with open(root_cargo_toml_path) as f:
    cargo_data = toml.load(f)
    version = cargo_data["workspace"]["package"]["version"]

CRATE_NAME = f"simba@{version}"
CRATE_DIR = "simba-core"
DOC_FILE = "doc/user_manual/docs/config_documentation.md"
CONFIG_SUFFIX = "Config"
ENTRY_CONFIG = "SimulatorConfig"
INDENT_TXT = "\t"
OPTIONAL = "Optional"
LIST = "List"
EXPANDED_FILE = "expanded.rs"

ENUMERATION_COMMENT = "Enum"

BASE_DOC_URL = "https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/"

ADDITIONNAL_CONFIG_ITEMS = ["Source", "Sort", "TimeMode", "LogLevel", "InternalLog"]

def main():
    # First, generate rustdoc JSON to get source file mappings
    print("Generating rustdoc JSON...")
    rustdoc_result = subprocess.run(
        ["cargo", "+nightly", "rustdoc", "--package", CRATE_NAME, "--", 
         "-Z", "unstable-options", "--output-format", "json"],
        capture_output=True,
        text=True,
        cwd=CRATE_DIR
    )
    if rustdoc_result.returncode != 0:
        print("ERROR: rustdoc JSON generation failed:", rustdoc_result.stderr)
        type_to_file = {}
        exit(1)
    else:
        # Load the JSON doc to build type->file mapping
        import json
        json_path = "target/doc/simba.json"
        try:
            with open(json_path) as f:
                rustdoc_data = json.load(f)
            type_to_file = {}
            for item_id, item in rustdoc_data.get("index", {}).items():
                if list(item["inner"].keys())[0] in ["struct", "enum"]:
                    name = item.get("name")
                    # Get source file path
                    span = item.get("span")
                    if span and name:
                        filename = span.get("filename", "")
                        # Convert absolute path to relative
                        if filename.startswith(os.path.abspath(CRATE_DIR)):
                            filename = os.path.relpath(filename, os.path.abspath(CRATE_DIR))
                        # Remove beginning of the filename to match CI
                        filename = re.sub(f"{CRATE_DIR}/*", "", filename).lstrip("/")
                        type_to_file[name] = filename
            print(f"Mapped {len(type_to_file)} types to source files")
        except Exception as e:
            print(f"ERROR: Could not load rustdoc JSON: {e}")
            type_to_file = {}
            exit(1)
    
    # Now expand macros for parsing
    print("Expanding macros...")
    configs = {}
    result = subprocess.run(
        ["cargo", "expand", "--package", CRATE_NAME],
        capture_output=True,
        text=True,
        cwd=CRATE_DIR
    )
    if result.returncode != 0:
        print("cargo expand failed:", result.stderr)
        exit(1)
    else:
        raw_sources = [(result.stdout.encode(), EXPANDED_FILE)]

    with open("expanded_tmp.rs", "w") as f:
        f.write(result.stdout)
    
    configs = {}
    parser = Parser(Language(tsrust.language()))
    
    def extract_configs_recursive(node, filename, configs):
        """Recursively traverse AST to find all struct and enum items"""
        if node.type == 'struct_item' or node.type == 'enum_item':
            name_node = node.child_by_field_name("name")
            if name_node:
                name = name_node.text.decode("utf-8")
                if CONFIG_SUFFIX in name or name in ADDITIONNAL_CONFIG_ITEMS:
                    # Use source file from rustdoc if available, fallback to expanded file
                    source_file = type_to_file.get(name, filename)
                    configs[name] = (node, source_file, False)
        
        # Recursively process children
        for child in node.children:
            extract_configs_recursive(child, filename, configs)
    
    for raw, filename in raw_sources:
        tree = parser.parse(raw)
        extract_configs_recursive(tree.root_node, filename, configs)

    if not ENTRY_CONFIG in configs.keys():
        print(f"ERROR: {ENTRY_CONFIG} was expected in {filename}")
        exit(1)
    
    with open(DOC_FILE, "w") as docfile:
        docfile.write("# Configuration file documentation")
        docfile.write("""
This documentation is auto-generated.

To get more information on the parameter, check the documentation (or follow the links).

List of parameters:\n\n""")
        document_config(configs[ENTRY_CONFIG][0], docfile, configs)
        
        print("Config documention written in", DOC_FILE)
    
    # expanded_path = os.path.join(os.path.dirname(CRATE), EXPANDED_FILE)
    # if os.path.exists(expanded_path):
    #     os.remove(expanded_path)
    
    
def convert_rust_type_to_yaml(type:str) -> str|None:
    match type:
        case "String" | "Path":
            return "String"
        case "bool":
            return "Boolean"
        case "Value":
            return "User-specific struct"
        case "f32" | "f64":
            return "Float"
        case "i8" | "i16" | "i32" | "i64":
            return "Integer"
        case "u8" | "u16" | "u32" | "u64":
            return "Unsigned Integer"
        case _:
            return None
    
def type_from_node(type_node, comments, configs) -> Tuple[List[str], str | None]:
    if type_node is None:
        return (comments, None)
    if type_node.type == "array_type":
        element_type = type_node.children[1].text.decode('utf-8')
        length = type_node.children[3].text.decode('utf-8')
        return (comments + [f"Array[{length}]"], element_type)
    if len(type_node.children) > 0:
        type = type_node.children[-1].text.decode("utf-8")
    else:
        type = type_node.text.decode("utf-8")
    if type != "":
        if type in configs:
            return (comments, type)
        converted_type = convert_rust_type_to_yaml(type)
        if not converted_type is None:
            return (comments, converted_type)
    type_field = type_node.child_by_field_name("type")
    if type_field is None:
        return (comments, type)
    
    type = type_field.text.decode("utf-8")
    if type != "":
        if type in configs:
            return (comments, type)
        converted_type = convert_rust_type_to_yaml(type)
        if not converted_type is None:
            return (comments, converted_type)
    if type == "Option":
        comments.append(OPTIONAL)
    if type == "Vec":
        comments.append(LIST)
    type_args = type_node.child_by_field_name("type_arguments")
    if type_args is None:
        return (comments, type)
    if len(type_args.children) > 3:
        print(f"More than 3 types: {type} => {type_args.children}")
    subtype = type_args.named_child(0)
    return type_from_node(subtype, comments, configs)

def url_of_type(type_name: str, type, configs) -> str :
    filename = configs[type_name][1]
    
    # Remove src/ prefix and .rs extension, replace / with ::
    if filename.startswith("src/"):
        filename = filename[4:]
    if filename.endswith(".rs"):
        filename = filename[:-3]
    
    # Convert path to module path: utils/python.rs -> utils/python
    module_path = filename.replace("/", "/")
    type_prefix = "struct"
    if type == "enum_item":
        type_prefix = "enum"
    return os.path.join(BASE_DOC_URL, module_path, f"{type_prefix}.{type_name}.html")
    
    
def document_config(node, docfile, configs, depth=0):
    indent = INDENT_TXT*depth
    body = node.child_by_field_name("body")
    for i in range(len(body.children)):
        n = body.children[i]
        
        # Struct
        if n.type == "field_declaration" or n.type == "enum_variant":
            child = n.child_by_field_name("name")
            name = child.text.decode("utf-8")
            non_type_enum = False
            if n.type == "field_declaration":
                type_field = n.child_by_field_name("type")
            else:
                type_field = n.child_by_field_name("body")
                if type_field is None:
                    non_type_enum = True
                else:
                    type_field = type_field.child_by_field_name("type") 
            comments, config_name = type_from_node(type_field, [], configs)
            next_node = None
            if not config_name is None:
                if config_name in configs:
                    next_node, filename, already_seen = configs[config_name]
                    comments.insert(0, f"[{config_name}]({url_of_type(config_name, next_node.type, configs)})")
                    if already_seen:
                        comments.insert(1, "See above")
                        next_node = None
                    else:
                        configs[config_name] = (next_node, filename, True)
                else:
                    comments.insert(0, config_name)
            if not next_node is None and next_node.type == "enum_item":
                comments.append(ENUMERATION_COMMENT)
            comments_str = ""
            if len(comments) > 0:
                comments_str += ", ".join(comments)
            if not config_name is None and config_name == "User-specific struct" and name == "config":
                docfile.write(f"{indent}- Insert {comments_str}\n")
            else:
                docfile.write(f"{indent}- `{name}`{':' if comments_str != '' else ''} {comments_str}\n")
            if not next_node is None:
                document_config(next_node, docfile, configs, depth+1)
            
        
if __name__ == "__main__":
    main()