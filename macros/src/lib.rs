use proc_macro::TokenStream;
use syn::parse_macro_input;

mod time_analysis_impl;

#[proc_macro_attribute]
pub fn time_analysis(attr: TokenStream, input: TokenStream) -> TokenStream {
    // let input = parse_macro_input!(input);
    println!("input: {}", input);
    // let attr = parse_macro_input!(attr);
    time_analysis_impl::time_analysis_impl(attr, input).into()
}
