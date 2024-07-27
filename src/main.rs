use pages::router::Router;

mod pages;
mod component;

fn main() {
    wasm_logger::init(wasm_logger::Config::default());
    yew::Renderer::<Router>::new().render();
}
