use yew::prelude::*;
use pages::router::Router;

mod pages;

fn main() {
    yew::Renderer::<Router>::new().render();
}
