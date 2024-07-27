use crate::component::preview::Preview;
use gloo_net::http::Request;
use log::info;
use yew::platform::*;
use yew::prelude::*;

pub struct Post {
    preview: Vec<Html>,
}

pub enum Msg {
    Update(Vec<String>),
}

impl Component for Post {
    type Message = Msg;
    type Properties = ();

    fn create(_ctx: &Context<Self>) -> Self {
        Self {
            preview: Vec::new(),
        }
    }

    fn update(&mut self, _ctx: &Context<Self>, msg: Self::Message) -> bool {
        match msg {
            Msg::Update(list) => {
                let preview: Vec<Html> = list
                    .iter()
                    .map(|filename| {
                        html! { <Preview filename={filename.clone()} /> }
                    })
                    .collect();
                self.preview = preview;
                true
            }
        }
    }

    fn view(&self, _ctx: &Context<Self>) -> Html {
        let preview = self.preview.clone();

        html! {
            <div class="post-wrapper">
                <div class="post-container">
                    <div class="post-list">
                        { for preview }
                    </div>
                    <div class="post-tag">
                        <div class="post-tag-title">
                            { "tag" }
                        </div>
                        <div class="post-tag-list">
                        </div>
                    </div>
                </div>
            </div>
        }
    }

    fn rendered(&mut self, _ctx: &Context<Self>, first_render: bool) {
        if first_render {
            let link = _ctx.link().clone();
            spawn_local(async move {
                match Request::get("markdown/list.json").send().await {
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
