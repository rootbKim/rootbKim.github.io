use super::{navbar::Navbar, post::Post};
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
        Route::Post => {
            html! { <Post /> }
        }
        Route::Page { filename } => {
            html! { <Page filename={filename.clone()} /> }
        }
        Route::NotFound => {
            html! { "Page Not Found" }
        }
    }
}

#[derive(Routable, PartialEq, Eq, Clone, Debug)]
pub enum Route {
    #[at("/")]
    Post,
    #[at("/page/:filename")]
    Page { filename: String },
    #[not_found]
    #[at("/404")]
    NotFound,
}
