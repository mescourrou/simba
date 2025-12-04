extern crate proc_macro;

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::{quote, ToTokens};
use syn::{Data, parse_macro_input, DeriveInput, spanned::Spanned};

#[proc_macro_derive(ToVec)]
pub fn derive_tovec(item: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(item as syn::DeriveInput);

    let struct_identifier = &input.ident;
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
                            "ToVec derive macro requires named fields"
                        ).to_compile_error().into();
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
                        field_enum_list.extend(quote! {#struct_identifier::#id { #(#field_defaults),* }});
                    }
                }
            }
            enum_impl = quote! {
                #[automatically_derived]
                impl crate::utils::enum_tools::ToVec<#struct_identifier> for #struct_identifier {
                    fn to_vec() -> Vec<#struct_identifier> {
                        vec![#field_enum_list]
                    }
                }
            }
        }
        Data::Union(_) => {
            return syn::Error::new(
                input.span(),
                "ToVec derive macro does not support unions"
            ).to_compile_error().into();
        }
    }

    quote! {
        #[automatically_derived]
        impl crate::utils::enum_tools::ToVec<&'static str> for #struct_identifier {
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

    let mut match_impl = TokenStream2::new();
    match &input.data {
        Data::Enum(syn::DataEnum { variants, .. }) => {
            if variants.is_empty() {
                return syn::Error::new(
                    input.span(),
                    "EnumToString requires at least one variant"
                ).to_compile_error().into();
            }
            for variant in variants {
                let id = &variant.ident;
                let id_str = id.to_string();
                
                match &variant.fields {
                    syn::Fields::Named(fields) => {
                        // Named fields: Variant { field1, field2, ... }
                        let field_names: Vec<_> = fields.named.iter()
                            .filter_map(|f| f.ident.as_ref())
                            .collect();
                        
                        if field_names.is_empty() {
                            match_impl.extend(quote! {
                                #struct_identifier::#id { .. } => #id_str.to_string(),
                            });
                        } else {
                            let debug_format = field_names.iter()
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
                                .map(|i| syn::Ident::new(&format!("field{}", i), proc_macro2::Span::call_site()))
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
                "EnumToString can only be derived for enums, not structs"
            ).to_compile_error().into();
        }
        Data::Union(_) => {
            return syn::Error::new(
                input.span(),
                "EnumToString can only be derived for enums, not unions"
            ).to_compile_error().into();
        }
    }

    quote! {
        #[automatically_derived]
        impl std::fmt::Display for #struct_identifier {
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

enum ConfigDerivesType {
    Struct,
    Enum,
    None,
}

/// Attribute macro that applies common derives for config types
/// Usage: #[config_derives] or #[config_derives(skip_check)]
#[proc_macro_attribute]
pub fn config_derives(attr: TokenStream, item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as DeriveInput);
    
    // Parse attributes to check for skip_check
    let attr_str = attr.to_string();

    let mut check_derive = quote! { config_checker::macros::Check, };
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
                check_derive = quote! { };
            }
            "skip_deserialize" => {
                deserialize_derive = quote! { };
            }
            "tag_content" => {
                tagged_derive = quote! {  #[serde(tag = "type", content = "config")] };
            }
            "untagged" => {
                tagged_derive = quote! {  #[serde(untagged)] };
            }
            "skip_unknown_fields" => {
                unknown_fields_derive = quote! { };
            }
            "skip_jsonschema" => {
                jsonschema_derive = quote! { };
                jsonschema_derive_struct = quote! { };
            }
            "" => {}
            _ => {
                return syn::Error::new(
                    input.span(),
                    format!("Unknown attribute '{}' for config_derives", trimmed)
                ).to_compile_error().into();
            }
        }
    }
   
    let struct_or_enum = match &input.data {
        Data::Struct(_) => {
            ConfigDerivesType::Struct
        }
        Data::Enum(_) => {
            ConfigDerivesType::Enum
        }
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