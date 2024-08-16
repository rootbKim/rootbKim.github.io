use crate::component::thumbnail::Thumbnail;
use gloo_net::http::Request;
use log::info;
use yew::platform::*;
use yew::prelude::*;
use super::router::Route;
use yew_router::prelude::*;

pub struct Portfolio {
    list: Vec<String>,
    thumbnail: Vec<Html>,
    tags: Vec<(String, u16)>,
    selected_tag: String,
}

pub enum Msg {
    Update(Vec<String>),
}

impl Component for Portfolio {
    type Message = Msg;
    type Properties = ();

    fn create(_ctx: &Context<Self>) -> Self {
        Self {
            list: Vec::new(),
            thumbnail: Vec::new(),
            tags: Vec::new(),
            selected_tag: String::new(),
        }
    }

    fn update(&mut self, _ctx: &Context<Self>, msg: Self::Message) -> bool {
        match msg {
            Msg::Update(list) => {
                self.list = list.clone();
                let thumbnail: Vec<Html> = list
                    .iter()
                    .rev()
                    .map(|filename| {
                        html! { 
                            <Link<Route> classes={classes!("portfolio-thumbnail")} to={Route::Page { class: "portfolio".to_string(), filename: filename.to_string() }}>
                                <Thumbnail filename={filename.clone()}/>
                            </Link<Route>>
                        }
                    })
                    .collect();
                self.thumbnail = thumbnail;
                true
            }
        }
    }

    fn view(&self, _ctx: &Context<Self>) -> Html {
        let thumbnail = self.thumbnail.clone();

        html! {
            <div class="portfolio-wrapper">
                <div class="portfolio-list">
                    { for thumbnail }
                </div>
            </div>
        }
    }

    fn rendered(&mut self, _ctx: &Context<Self>, first_render: bool) {
        if first_render {
            let link = _ctx.link().clone();
            spawn_local(async move {
                match Request::get("/portfolio/list.json").send().await {
                    Ok(resp) => match resp.text().await {
                        Ok(text) => match serde_json::from_str::<Vec<String>>(&text) {
                            Ok(list) => {
                                link.send_message(Msg::Update(list));
                            }
                            Err(_) => {
                                info!("list error");
                            }
                        },
                        Err(_) => {
                            info!("list error");
                        }
                    },
                    Err(_) => {
                        info!("list error");
                    }
                }
            });
        }
    }
}
