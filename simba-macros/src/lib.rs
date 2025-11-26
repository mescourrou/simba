extern crate proc_macro;

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::{quote, ToTokens};
use syn::Data;

#[proc_macro_derive(ToVec)]
pub fn derive_tovec(item: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(item as syn::DeriveInput);

    let struct_identifier = &input.ident;

    let mut field_str_list = TokenStream2::new();
    let mut field_enum_list = TokenStream2::new();
    let mut first = true;
    let mut enum_impl = TokenStream2::new();
    match &input.data {
        Data::Struct(syn::DataStruct { fields, .. }) => {
            for f in fields {
                let id = f.ident.clone();
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
            let mut at_least_one_not_complex = false;
            for f in variants {
                let id = f.ident.clone();
                let id_str = id.to_token_stream().to_string();
                if first {
                    first = false;
                } else {
                    field_str_list.extend(quote! {, });
                    field_enum_list.extend(quote! {, });
                }
                field_str_list.extend(quote! {#id_str});
                if f.fields.is_empty() {
                    at_least_one_not_complex = true;
                    field_enum_list.extend(quote! {#struct_identifier::#id});
                }
            }
            if at_least_one_not_complex {
                enum_impl = quote! {
                    #[automatically_derived]
                    impl crate::utils::enum_tools::ToVec<#struct_identifier> for #struct_identifier {
                        fn to_vec() -> Vec<#struct_identifier> {
                            vec![#field_enum_list]
                        }
                    }
                }
            }
        }
        _ => unimplemented!(),
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
            for f in variants {
                let id = f.ident.clone();
                let id_str = id.to_token_stream().to_string();
                if f.fields.is_empty() {
                    match_impl.extend(quote! {#struct_identifier::#id => #id_str,
                    });
                } else {
                    match_impl.extend(quote! {#struct_identifier::#id(_) => #id_str,
                    });
                }
            }
        }
        _ => unimplemented!(),
    }

    quote! {
        #[automatically_derived]
        impl std::fmt::Display for #struct_identifier {
            #[allow(unreachable_patterns)]
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "{}", match &self {
                        #match_impl
                        _ => unimplemented!()
                    })
            }
        }

    }
    .into()
}
