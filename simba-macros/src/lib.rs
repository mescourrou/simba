extern crate proc_macro;

use std::collections::HashMap;

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::{ToTokens, format_ident, quote};
use syn::{Data, DeriveInput, parse::Parse, parse_macro_input, spanned::Spanned};

#[proc_macro_derive(ToVec)]
pub fn derive_tovec(item: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(item as syn::DeriveInput);

    let struct_identifier = &input.ident;

    let generics = &input.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

    let mut field_enum_list = TokenStream2::new();
    let mut enum_impl = TokenStream2::new();
    let mut field_str_list = TokenStream2::new();
    let mut first = true;
    match &input.data {
        Data::Struct(syn::DataStruct { fields, .. }) => {
            for f in fields {
                let id = match &f.ident {
                    Some(ident) => ident.clone(),
                    None => {
                        return syn::Error::new(
                            f.span(),
                            "ToVec derive macro requires named fields",
                        )
                        .to_compile_error()
                        .into();
                    }
                };
                let id_str = id.to_token_stream().to_string();
                if first {
                    first = false;
                } else {
                    field_str_list.extend(quote! {, });
                }
                field_str_list.extend(quote! {#id_str});
            }
        }
        Data::Enum(syn::DataEnum { variants, .. }) => {
            let mut enum_first = true;
            for f in variants {
                let id = f.ident.clone();
                let id_str = id.to_token_stream().to_string();
                if first {
                    first = false;
                } else {
                    field_str_list.extend(quote! {, });
                }
                field_str_list.extend(quote! {#id_str});

                if enum_first {
                    enum_first = false;
                } else {
                    field_enum_list.extend(quote! {, });
                }

                // Generate variant construction with default values for fields
                match &f.fields {
                    syn::Fields::Unit => {
                        field_enum_list.extend(quote! {#struct_identifier::#id});
                    }
                    syn::Fields::Unnamed(fields) => {
                        let defaults = fields.unnamed.iter().map(|field| {
                            let ty = &field.ty;
                            quote! { <#ty>::default() }
                        });
                        field_enum_list.extend(quote! {#struct_identifier::#id(#(#defaults),*)});
                    }
                    syn::Fields::Named(fields) => {
                        let field_defaults = fields.named.iter().map(|field| {
                            let name = field.ident.as_ref().unwrap();
                            let ty = &field.ty;
                            quote! { #name: <#ty>::default() }
                        });
                        field_enum_list
                            .extend(quote! {#struct_identifier::#id { #(#field_defaults),* }});
                    }
                }
            }
            enum_impl = quote! {
                #[automatically_derived]
                impl #impl_generics crate::utils::enum_tools::ToVec<#struct_identifier #ty_generics> for #struct_identifier #ty_generics #where_clause {
                    fn to_vec() -> Vec<#struct_identifier #ty_generics> {
                        vec![
                            #field_enum_list
                            ]
                    }
                }
            }
        }
        Data::Union(_) => {
            return syn::Error::new(input.span(), "ToVec derive macro does not support unions")
                .to_compile_error()
                .into();
        }
    }

    quote! {
        #[automatically_derived]
        impl #impl_generics crate::utils::enum_tools::ToVec<&'static str> for #struct_identifier #ty_generics #where_clause {
            fn to_vec() -> Vec<&'static str> {
                vec![#field_str_list]
            }
        }

        #enum_impl
    }
    .into()
}

#[proc_macro_derive(EnumToString)]
pub fn derive_enum_to_string(item: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(item as syn::DeriveInput);

    let struct_identifier = &input.ident;
    let generics = &input.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

    let mut match_impl = TokenStream2::new();
    match &input.data {
        Data::Enum(syn::DataEnum { variants, .. }) => {
            if variants.is_empty() {
                return syn::Error::new(input.span(), "EnumToString requires at least one variant")
                    .to_compile_error()
                    .into();
            }
            for variant in variants {
                let id = &variant.ident;
                let id_str = id.to_string();

                match &variant.fields {
                    syn::Fields::Named(fields) => {
                        // Named fields: Variant { field1, field2, ... }
                        let field_names: Vec<_> = fields
                            .named
                            .iter()
                            .filter_map(|f| f.ident.as_ref())
                            .collect();

                        if field_names.is_empty() {
                            match_impl.extend(quote! {
                                #struct_identifier::#id { .. } => #id_str.to_string(),
                            });
                        } else {
                            let debug_format = field_names
                                .iter()
                                .map(|fname| format!("{}: {{:?}}", fname))
                                .collect::<Vec<_>>()
                                .join(", ");
                            let format_str = format!("{} {{ {} }}", id_str, debug_format);

                            match_impl.extend(quote! {
                                #struct_identifier::#id { #(#field_names),* } => {
                                    format!(#format_str, #(#field_names),*)
                                },
                            });
                        }
                    }
                    syn::Fields::Unnamed(fields) => {
                        // Unnamed fields: Variant(field1, field2, ...)
                        let field_count = fields.unnamed.len();

                        if field_count == 1 {
                            // Single field: format as "Variant(value)"
                            match_impl.extend(quote! {
                                #struct_identifier::#id(inner) => {
                                    format!("{}({:?})", #id_str, inner)
                                },
                            });
                        } else {
                            // Multiple fields: format as "Variant(val1, val2, ...)"
                            let field_indices: Vec<_> = (0..field_count)
                                .map(|i| {
                                    syn::Ident::new(
                                        &format!("field{}", i),
                                        proc_macro2::Span::call_site(),
                                    )
                                })
                                .collect();
                            let debug_placeholders = vec!["{:?}"; field_count].join(", ");
                            let format_str = format!("{}({})", id_str, debug_placeholders);

                            match_impl.extend(quote! {
                                #struct_identifier::#id(#(#field_indices),*) => {
                                    format!(#format_str, #(#field_indices),*)
                                },
                            });
                        }
                    }
                    syn::Fields::Unit => {
                        // Unit variant: Variant
                        match_impl.extend(quote! {
                            #struct_identifier::#id => #id_str.to_string(),
                        });
                    }
                }
            }
        }
        Data::Struct(_) => {
            return syn::Error::new(
                input.span(),
                "EnumToString can only be derived for enums, not structs",
            )
            .to_compile_error()
            .into();
        }
        Data::Union(_) => {
            return syn::Error::new(
                input.span(),
                "EnumToString can only be derived for enums, not unions",
            )
            .to_compile_error()
            .into();
        }
    }

    quote! {
        #[automatically_derived]
        impl #impl_generics std::fmt::Display for #struct_identifier #ty_generics #where_clause {
            #[allow(unreachable_patterns)]
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "{}", match self {
                        #match_impl
                    })
            }
        }

    }
    .into()
}

fn parse_ui_name(attrs: &[syn::Attribute]) -> syn::Result<Option<String>> {
    for attr in attrs {
        if !attr.path().is_ident("ui_name") {
            continue;
        }

        match &attr.meta {
            // #[ui_name = "MyName"]
            syn::Meta::NameValue(nv) => {
                if let syn::Expr::Lit(expr_lit) = &nv.value
                    && let syn::Lit::Str(s) = &expr_lit.lit
                {
                    return Ok(Some(s.value()));
                }
                return Err(syn::Error::new_spanned(
                    nv,
                    r#"expected: #[ui_name = "..."]"#,
                ));
            }
            _ => {
                return Err(syn::Error::new_spanned(
                    attr,
                    r#"expected: #[ui_name = "..."]"#,
                ));
            }
        }
    }

    Ok(None)
}

fn parse_ui_show_all(attrs: &[syn::Attribute]) -> syn::Result<Option<String>> {
    for attr in attrs {
        if !attr.path().is_ident("show_all") {
            continue;
        }

        match &attr.meta {
            // #[show_all = "Elements"]
            syn::Meta::NameValue(nv) => {
                if let syn::Expr::Lit(expr_lit) = &nv.value
                    && let syn::Lit::Str(s) = &expr_lit.lit
                {
                    return Ok(Some(s.value()));
                }
                return Err(syn::Error::new_spanned(
                    nv,
                    r#"expected: #[show_all = "..."]"#,
                ));
            }
            syn::Meta::List(ml) => {
                return Ok(Some(ml.tokens.to_string()));
            }
            syn::Meta::Path(_) => {
                return Err(syn::Error::new_spanned(
                    attr,
                    r#"expected: #[show_all] or #[show_all = "..."]"#,
                ));
            }
        }
    }

    Ok(None)
}

#[proc_macro_derive(UIComponent, attributes(show_all))]
pub fn derive_ui_component(item: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(item as syn::DeriveInput);

    let struct_identifier = &input.ident;
    let generics = &input.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();
    let do_show_all = parse_ui_show_all(&input.attrs).expect("Failed to parse show_all attribute");

    let mut show_mut_match_impl = TokenStream2::new();
    let mut show_match_impl = TokenStream2::new();
    let mut add_match_impl = TokenStream2::new();
    match &input.data {
        Data::Enum(syn::DataEnum { variants, .. }) => {
            if variants.is_empty() {
                return syn::Error::new(input.span(), "UIComponent requires at least one variant")
                    .to_compile_error()
                    .into();
            }
            for variant in variants {
                let id = &variant.ident;
                let id_str = parse_ui_name(&variant.attrs)
                    .expect("Failed to parse UI name")
                    .unwrap_or_else(|| id.to_string());

                match &variant.fields {
                    syn::Fields::Named(fields) => {
                        // Named fields: Variant { field1, field2, ... }
                        let field_names: Vec<_> = fields
                            .named
                            .iter()
                            .filter_map(|f| f.ident.as_ref())
                            .collect();
                        if !fields.named.is_empty() {
                            show_mut_match_impl.extend(quote! {
                                Self::#id(cfg) => cfg.show_mut(
                                    ui,
                                    ctx,
                                    buffer_stack,
                                    global_config,
                                    current_node_name,
                                    unique_id,
                                ),
                            });
                            show_match_impl.extend(quote! {
                                Self::#id(cfg) => cfg.show(
                                    ui,
                                    ctx,
                                    unique_id,
                                ),
                            });
                            if do_show_all.is_some() {
                                let default_fields = field_names.iter().map(|fname| {
                                    quote! { #fname: Default::default() }
                                });
                                add_match_impl.extend(quote! {
                                    #id_str => {
                                        config_list.push(Self::#id(#{ #(#default_fields),* }));
                                    },
                                });
                            }
                        } else {
                            show_mut_match_impl.extend(quote! {
                                Self::#id => {}
                            });
                            show_match_impl.extend(quote! {
                                Self::#id => {}
                            });
                            if do_show_all.is_some() {
                                add_match_impl.extend(quote! {
                                    #id_str => {
                                        config_list.push(Self::#id);
                                    },
                                });
                            }
                        }
                    }
                    syn::Fields::Unnamed(fields) => {
                        // // Unnamed fields: Variant(field1, field2, ...)
                        if !fields.unnamed.is_empty() {
                            show_mut_match_impl.extend(quote! {
                                Self::#id(cfg) => cfg.show_mut(
                                    ui,
                                    ctx,
                                    buffer_stack,
                                    global_config,
                                    current_node_name,
                                    unique_id,
                                ),
                            });
                            show_match_impl.extend(quote! {
                                Self::#id(cfg) => cfg.show(
                                    ui,
                                    ctx,
                                    unique_id,
                                ),
                            });
                            if do_show_all.is_some() {
                                let default_fields = fields.unnamed.iter().map(|_| {
                                    quote! { Default::default() }
                                });
                                add_match_impl.extend(quote! {
                                    #id_str => {
                                        config_list.push(Self::#id(#(#default_fields),*));
                                    },
                                });
                            }
                        } else {
                            show_mut_match_impl.extend(quote! {
                                Self::#id => {}
                            });
                            show_match_impl.extend(quote! {
                                Self::#id => {}
                            });
                            if do_show_all.is_some() {
                                add_match_impl.extend(quote! {
                                    #id_str => {
                                        config_list.push(Self::#id);
                                    },
                                });
                            }
                        }
                    }
                    syn::Fields::Unit => {
                        // Unit variant: Variant
                        show_mut_match_impl.extend(quote! {
                            #struct_identifier::#id => {}
                        });
                        show_match_impl.extend(quote! {
                            Self::#id => {}
                        });
                        if do_show_all.is_some() {
                            add_match_impl.extend(quote! {
                                #id_str => {
                                    config_list.push(Self::#id);
                                },
                            });
                        }
                    }
                }
            }
        }
        Data::Struct(_) => {
            return syn::Error::new(
                input.span(),
                "UIComponent can only be derived for enums, not structs (for now)",
            )
            .to_compile_error()
            .into();
        }
        Data::Union(_) => {
            return syn::Error::new(
                input.span(),
                "UIComponent can only be derived for enums, not unions",
            )
            .to_compile_error()
            .into();
        }
    }

    let show_all_quote = if let Some(show_all_name) = do_show_all {
        let show_all_name = show_all_name.trim_matches('"').to_owned() + ":";
        quote! {
            #[cfg(feature = "gui")]
            impl #impl_generics #struct_identifier #ty_generics #where_clause {
                pub fn show_all_mut(
                    config_list: &mut Vec<Self>,
                    ui: &mut egui::Ui,
                    ctx: &egui::Context,
                    buffer_stack: &mut std::collections::BTreeMap<String, String>,
                    global_config: &SimulatorConfig,
                    current_node_name: Option<&String>,
                    unique_id: &str,
                ) {
                    use crate::{gui::utils::string_combobox, utils::enum_tools::ToVec};

                    ui.label(#show_all_name);
                    let mut to_remove = None;
                    for (i, element) in config_list.iter_mut().enumerate() {
                        ui.horizontal_top(|ui| {
                            let unique_fault_id = format!("{}-{i}-{unique_id}", stringify!(#struct_identifier));
                            element.show_mut(
                                ui,
                                ctx,
                                buffer_stack,
                                global_config,
                                current_node_name,
                                &unique_fault_id,
                            );

                            if ui.button("X").clicked() {
                                to_remove = Some(i);
                            }
                        });
                    }
                    if let Some(i) = to_remove {
                        config_list.remove(i);
                    }

                    ui.horizontal(|ui| {
                        let buffer_key = format!("selected-new-{}-{unique_id}", stringify!(#struct_identifier));
                        if !buffer_stack.contains_key(&buffer_key) {
                            buffer_stack.insert(buffer_key.clone(), #struct_identifier::default().to_string());
                        }
                        string_combobox(
                            ui,
                            &Self::to_vec(),
                            buffer_stack.get_mut(&buffer_key).unwrap(),
                            format!("{}-choice-{}", stringify!(#struct_identifier), unique_id),
                        );
                        if ui.button("Add").clicked() {
                            let selected_fault = buffer_stack.get(&buffer_key).unwrap();
                            match selected_fault.as_str() {
                                #add_match_impl
                                _ => panic!("Where did you find this {}?", stringify!(#struct_identifier)),
                            };
                        }
                    });
                }

                pub fn show_all(
                    config_list: &[Self],
                    ui: &mut egui::Ui,
                    ctx: &egui::Context,
                    unique_id: &str,
                ) {
                    ui.label(#show_all_name);
                    for (i, config) in config_list.iter().enumerate() {
                        ui.horizontal_top(|ui| {
                            let unique_config_id = format!("{}-{i}-{unique_id}", stringify!(#struct_identifier));
                            config.show(ui, ctx, &unique_config_id);
                        });
                    }
                }
            }
        }
    } else {
        TokenStream2::new()
    };

    quote! {
        #[automatically_derived]
        #[cfg(feature = "gui")]
        impl #impl_generics crate::gui::UIComponent for  #struct_identifier #ty_generics #where_clause  {
            fn show_mut(
                &mut self,
                ui: &mut egui::Ui,
                ctx: &egui::Context,
                buffer_stack: &mut std::collections::BTreeMap<String, String>,
                global_config: &SimulatorConfig,
                current_node_name: Option<&String>,
                unique_id: &str,
            ) {
                ui.vertical(|ui| {
                    let self_str = self.to_string();
                    egui::CollapsingHeader::new(self_str)
                        .id_salt(format!("{}-{}", stringify!(#struct_identifier), unique_id))
                        .show(ui, |ui| {
                            match self {
                                #show_mut_match_impl
                            };
                        });
                });

            }

            fn show(&self, ui: &mut egui::Ui, ctx: &egui::Context, unique_id: &str) {
                let self_str = self.to_string();
                egui::CollapsingHeader::new(self_str)
                    .id_salt(format!("{}-{}", stringify!(#struct_identifier), unique_id))
                    .show(ui, |ui| {
                        match self {
                            #show_match_impl
                        };
                    });
            }
        }

        #show_all_quote
    }
    .into()
}

enum ConfigDerivesType {
    Struct,
    Enum,
    None,
}

/// Attribute macro that applies common derives for config types
/// Usage: #[config_derives]
/// Options:
/// - skip_check: do not derive Check (for configs that cannot be checked with the current checkers, such as fault models)
/// - skip_deserialize: do not derive Deserialize (for configs that cannot be deserialized, such as those containing trait objects)
/// - tag_content: for enums, use #[serde(tag = "type", content = "value")] instead of #[serde(tag = "type")]
/// - untagged: for enums, use #[serde(untagged)] instead of #[serde(tag = "type")]
/// - skip_unknown_fields: do not use #[serde(deny_unknown_fields)] (for configs that want to allow unknown fields, such as fault models that can have custom parameters)
/// - skip_jsonschema: do not derive JsonSchema (for configs that cannot be represented in JSON Schema, such as those containing trait objects)
///
/// Example: #[config_derives(skip_check, skip_deserialize, tag_content)]
#[proc_macro_attribute]
pub fn config_derives(attr: TokenStream, item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as DeriveInput);

    // Parse attributes to check for skip_check
    let attr_str = attr.to_string();

    let mut check_derive = quote! { Check, };
    let mut deserialize_derive = quote! { serde::Deserialize, };
    let mut tagged_derive = quote! { #[serde(tag = "type")] };
    let mut jsonschema_derive = quote! {
        #[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
    };
    let mut jsonschema_derive_struct = quote! {
        #[cfg_attr(feature = "schema", schemars(default))]
    };
    let mut unknown_fields_derive = quote! {
        #[serde(deny_unknown_fields)]
    };
    for attribute in attr_str.split(',') {
        let trimmed = attribute.trim();
        match trimmed {
            "skip_check" => {
                check_derive = quote! {};
            }
            "skip_deserialize" => {
                deserialize_derive = quote! {};
            }
            "tag_content" => {
                tagged_derive = quote! {  #[serde(tag = "type", content = "value")] };
            }
            "untagged" => {
                tagged_derive = quote! {  #[serde(untagged)] };
            }
            "skip_unknown_fields" => {
                unknown_fields_derive = quote! {};
            }
            "skip_jsonschema" => {
                jsonschema_derive = quote! {};
                jsonschema_derive_struct = quote! {};
            }
            "" => {}
            _ => {
                return syn::Error::new(
                    input.span(),
                    format!("Unknown attribute '{}' for config_derives", trimmed),
                )
                .to_compile_error()
                .into();
            }
        }
    }

    let struct_or_enum = match &input.data {
        Data::Struct(_) => ConfigDerivesType::Struct,
        Data::Enum(_) => ConfigDerivesType::Enum,
        _ => ConfigDerivesType::None,
    };

    // Conditional attributes for structs vs enums
    let type_only_attrs = if let ConfigDerivesType::Struct = struct_or_enum {
        quote! {
            #jsonschema_derive_struct
            #[serde(default)]
        }
    } else if let ConfigDerivesType::Enum = struct_or_enum {
        quote! {
            #[derive(simba_macros::EnumToString)]
            #tagged_derive
        }
    } else {
        quote! {}
    };

    // Standard derives for config types
    let output = quote! {
        use config_checker::*;
        #[derive(
            serde::Serialize,
            #deserialize_derive
            Debug,
            Clone,
            PartialEq,
            #check_derive
            simba_macros::ToVec,
        )]
        #jsonschema_derive
        #unknown_fields_derive
        #type_only_attrs
        #input
    };

    output.into()
}

struct EnumVariablesInput {
    enum_name: syn::Ident,
    variants: Vec<EnumVariablesVariant>,
}

impl Parse for EnumVariablesInput {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let enum_name: syn::Ident = input.parse()?;
        input.parse::<syn::Token![;]>()?;

        let mut variants = Vec::new();
        while !input.is_empty() {
            variants.push(input.parse()?);
            if !input.is_empty() {
                input.parse::<syn::Token![;]>()?;
            }
        }

        Ok(EnumVariablesInput {
            enum_name,
            variants,
        })
    }
}

struct EnumVariablesVariant {
    sub_enums: Vec<syn::Ident>,
    variant_name: syn::Ident,
    value: String,
    additional_values: Vec<String>,
}

impl Parse for EnumVariablesVariant {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        let mut sub_enums = Vec::new();
        while input.peek(syn::Ident) && !input.peek(syn::Token![:]) {
            sub_enums.push(input.parse()?);
            if input.peek(syn::Token![,]) {
                input.parse::<syn::Token![,]>()?;
            }
        }

        let variant_name = if input.peek(syn::Token![:]) {
            // There were sub enums
            input.parse::<syn::Token![:]>()?;
            let name = input.parse()?;
            input.parse::<syn::Token![,]>()?;
            name
        } else {
            // No sub enums, the first identifier is the variant name
            // Get the name from the sub enum list
            sub_enums.pop().unwrap()
        };
        let value_lit: syn::LitStr = input.parse()?;
        let value = value_lit.value();

        let mut additional_values = Vec::new();
        while !input.peek(syn::Token![;]) && !input.is_empty() {
            input.parse::<syn::Token![,]>()?;
            if input.peek(syn::LitStr) {
                let additional_value_lit: syn::LitStr = input.parse()?;
                additional_values.push(additional_value_lit.value());
            } else {
                break;
            }
        }

        Ok(EnumVariablesVariant {
            sub_enums,
            variant_name,
            value,
            additional_values,
        })
    }
}

#[proc_macro]
pub fn enum_variables(input: TokenStream) -> TokenStream {
    let parsed = parse_macro_input!(input as EnumVariablesInput);
    let mut sub_enums = HashMap::new();
    for variant in &parsed.variants {
        for sub_enum in &variant.sub_enums {
            if !sub_enums.contains_key(sub_enum) {
                sub_enums.insert(sub_enum.clone(), Vec::new());
            }
            sub_enums.get_mut(sub_enum).unwrap().push(variant);
        }
    }

    let mut meta_enum = TokenStream2::new();
    let mut generated_sub_enums = TokenStream2::new();
    if !sub_enums.is_empty() {
        // meta_enum.extend(quote! {
        //     #enum_macros
        //     pub enum parse::parse! [<#enum_name Meta>] {
        // });
        for (meta_variant, sub_enum_variants) in sub_enums.iter() {
            let sub_enum_name = format_ident!("{}{}", parsed.enum_name, *meta_variant);
            meta_enum.extend(quote! {
                #meta_variant (#sub_enum_name),
            });

            let mut sub_enum_variants_tokens = TokenStream2::new();
            for variant in sub_enum_variants {
                let variant_name = &variant.variant_name;
                let value = &variant.value;
                let additional_values = &variant.additional_values;
                sub_enum_variants_tokens
                    .extend(quote! {#variant_name, #value #(, #additional_values)*;});
            }
            generated_sub_enums.extend(quote! {
                crate::utils::enum_tools::__enum_variables_emit_subenum!(
                    #sub_enum_name;
                    #sub_enum_variants_tokens
                );

            });
        }
        let meta_enum_name = format_ident!(
            "{}{}",
            parsed.enum_name,
            syn::Ident::new("Meta", proc_macro2::Span::call_site())
        );
        meta_enum = quote! {
            // #enum_macros
            pub enum #meta_enum_name {
                #meta_enum
            }
        };
    }

    let enum_name = parsed.enum_name;
    let mut enum_variants_tokens = TokenStream2::new();
    for variant in parsed.variants {
        let variant_name = &variant.variant_name;
        let value = &variant.value;
        let additional_values = &variant.additional_values;
        enum_variants_tokens.extend(quote! {#variant_name, #value #(, #additional_values)*;});
    }
    generated_sub_enums.extend(quote! {
        crate::utils::enum_tools::__enum_variables_emit_subenum!(
            #enum_name;
            #enum_variants_tokens
        );

    });

    quote! {
        #meta_enum

        #generated_sub_enums
    }
    .into()
}
