use super::{archive::Archive, home::Home, navbar::Navbar, post::Post, search::Search};
use crate::component::page::Page;
use log::info;
use yew::prelude::*;
use yew_router::prelude::*;

pub struct Router {}

impl Component for Router {
    type Message = ();
    type Properties = ();

    fn create(_ctx: &Context<Self>) -> Self {
        Self {}
    }

    fn view(&self, _ctx: &Context<Self>) -> Html {
        html! {
            <BrowserRouter>
                <Navbar />
                <div class="main-wrapper">
                    <div class="main-content">
                        <Switch<Route> render={switch} />
                    </div>
                </div>
            </BrowserRouter>
        }
    }
}

fn switch(routes: Route) -> Html {
    match routes {
        Route::Home => {
            html! { <Home /> }
        }
        Route::Post => {
            html! { <Post /> }
        }
        Route::Archive => {
            html! { <Archive /> }
        }
        Route::Page { class, filename } => {
            html! { <Page class={class.clone()} filename={filename.clone()} /> }
        }
        Route::Search => {
            html! { <Search /> }
        }
        Route::NotFound => {
            html! { "Page Not Found" }
        }
    }
}

#[derive(Routable, PartialEq, Eq, Clone, Debug)]
pub enum Route {
    #[at("/")]
    Home,
    #[at("/post/")]
    Post,
    #[at("/archive/")]
    Archive,
    #[at("/:class/:filename")]
    Page { class: String, filename: String },
    #[at("/search/")]
    Search,
    #[not_found]
    #[at("/404")]
    NotFound,
}
