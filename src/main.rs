use pages::router::Router;

mod component;
mod pages;

fn main() {
    wasm_logger::init(wasm_logger::Config::default());
    yew::Renderer::<Router>::new().render();
}
