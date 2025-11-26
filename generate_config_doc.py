#!/bin/env python3

import tree_sitter_rust as tsrust
from tree_sitter import Language, Parser
import os
from typing import List, Tuple

import IPython

SOURCE_DIR = "simba-core/src"
DOC_FILE = "doc/user_manual/docs/config_documentation.md"
CONFIG_SUFFIX = "Config"
ENTRY_CONFIG = "SimulatorConfig"
INDENT_TXT = "\t"
OPTIONAL = "Optional"
LIST = "List"

ENUMERATION_COMMENT = "Enum"

BASE_DOC_URL = "https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/"

ADDITIONNAL_CONFIG_ITEMS = ["Source", "Sort", "TimeMode", "LogLevel", "InternalLog"]

def main():
    configs = {}
    for root, dir, files in os.walk(SOURCE_DIR):
        parser = Parser(Language(tsrust.language()))
        for filename in files:
            filename = os.path.join(root, filename)
            with open(filename, mode = "rb") as file:
                raw = file.read()
            tree = parser.parse(raw)
            
            configs.update({
                node.children_by_field_name("name")[0].text.decode("utf-8"): (node, filename, False)
                for node in tree.root_node.children
                if (node.type == 'struct_item' or node.type == 'enum_item') and (CONFIG_SUFFIX in node.child_by_field_name("name").text.decode("utf-8") or node.child_by_field_name("name").text.decode("utf-8") in ADDITIONNAL_CONFIG_ITEMS)
            })
    
    if not ENTRY_CONFIG in configs.keys():
        print(f"ERROR: {ENTRY_CONFIG} was expected in {filename}")
    
    with open(DOC_FILE, "w") as docfile:
        docfile.write("# Configuration file documentation")
        docfile.write("""
This documentation is auto-generated.

To get more information on the parameter, check the documentation (or follow the links).

List of parameters:\n\n""")
        document_config(configs[ENTRY_CONFIG][0], docfile, configs)
        
        print("Config documention written in", DOC_FILE)
    
    
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
    filename = filename[len(SOURCE_DIR)+1:-3]
    type_prefix = "struct"
    if type == "enum_item":
        type_prefix = "enum"
    return os.path.join(BASE_DOC_URL, filename, f"{type_prefix}.{type_name}.html")
    
    
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
            if n.type == "enum_variant":
                docfile.write(f"{indent}- `!{name}`{':' if comments_str != '' else ''} {comments_str}\n")
            else:
                docfile.write(f"{indent}- `{name}`{':' if comments_str != '' else ''} {comments_str}\n")
            if not next_node is None:
                document_config(next_node, docfile, configs, depth+1)
            
        
if __name__ == "__main__":
    main()