use proc_macro::TokenStream;
use quote::{quote, ToTokens, TokenStreamExt};
use syn::parse_macro_input;

pub fn time_analysis_impl(attr: TokenStream, input: TokenStream) -> TokenStream {
    // let mut extended = quote!(
    //     print("begin of time analysis");
    // );
    let mut item: syn::Item = syn::parse(input).unwrap();
    let fn_item = match &mut item {
        syn::Item::Fn(fn_item) => fn_item,
        _ => panic!("expected fn"),
    };
    fn_item.block.stmts.insert(
        0,
        syn::parse(quote!(println!("begining");).into()).unwrap(),
    );
    fn_item.block.stmts.push(
        syn::parse(quote!(println!("end");).into()).unwrap(),
    );

    item.into_token_stream().into()
}
