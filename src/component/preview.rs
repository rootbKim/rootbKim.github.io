use crate::component::metadata::{extract_yaml_block_and_rest, parse_yaml, Metadata};
use gloo_net::http::Request;
use log::info;
use yew::platform::*;
use yew::prelude::*;

pub struct Preview {
    metadata: Metadata,
}

#[derive(Properties, PartialEq)]
pub struct Props {
    pub filename: String,
}

pub enum Msg {
    Update(String),
}

impl Component for Preview {
    type Message = Msg;
    type Properties = Props;

    fn create(_ctx: &Context<Self>) -> Self {
        Self {
            metadata: Metadata::default(),
        }
    }

    fn update(&mut self, _ctx: &Context<Self>, msg: Self::Message) -> bool {
        match msg {
            Msg::Update(text) => {
                let (yaml_block, _) = extract_yaml_block_and_rest(&text);

                match yaml_block {
                    Some(block) => match parse_yaml(block) {
                        Ok(metadata) => {
                            self.metadata = metadata;
                        }
                        Err(e) => info!("Error parsing YAML: {:?}", e),
                    },
                    None => info!("No YAML block found."),
                }
                true
            }
        }
    }

    fn view(&self, _ctx: &Context<Self>) -> Html {
        let title = self.metadata.title.clone().unwrap_or_default();
        let date = self.metadata.date.clone().unwrap_or_default();
        let excerpt = self.metadata.excerpt.clone().unwrap_or_default();

        html! {
            <>
                <div class="preview">
                    <div class="preview-title">
                        { title }
                    </div>
                    <div class="preview-date">
                        { date }
                    </div>
                    <div class="preview-excerpt">
                        { excerpt }
                    </div>
                    <div class="preview-tags">
                    </div>
                </div>
            </>
        }
    }

    fn rendered(&mut self, _ctx: &Context<Self>, first_render: bool) {
        if first_render {
            let link = _ctx.link().clone();
            let url = format!("/posts/{}", _ctx.props().filename);
            spawn_local(async move {
                match Request::get(url.as_str()).send().await {
                    Ok(resp) => match resp.text().await {
                        Ok(text) => {
                            link.send_message(Msg::Update(text));
                        }
                        Err(_) => {
                            info!("text error");
                        }
                    },
                    Err(_) => {
                        info!("text error");
                    }
                }
            });
        }
    }
}
