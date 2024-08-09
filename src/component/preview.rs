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
    pub class: String,
    pub filename: String,
    pub tag_cb: Callback<Vec<String>>,
    pub selected_tag: String,
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
                            let tags = self.metadata.tags.clone();
                            if !tags.is_empty() {
                                _ctx.props().tag_cb.emit(tags);
                            }
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
        if self.metadata.tags.contains(&_ctx.props().selected_tag)
            || _ctx.props().selected_tag == "".to_string()
        {
            let title = self.metadata.title.clone().unwrap_or_default();
            let date = self.metadata.date.clone().unwrap_or_default();
            let excerpt = self.metadata.excerpt.clone().unwrap_or_default();

            let tags: Vec<Html> = self
                .metadata
                .tags
                .clone()
                .iter()
                .rev()
                .map(|tag| {
                    html! {
                        <div class="tag">
                            {"#"}{ tag }
                        </div>
                    }
                })
                .collect();

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
                            { tags }
                        </div>
                    </div>
                </>
            }
        } else {
            html! {}
        }
    }

    fn rendered(&mut self, _ctx: &Context<Self>, first_render: bool) {
        if first_render {
            let link = _ctx.link().clone();
            let url = format!("/{}/{}.md", _ctx.props().class, _ctx.props().filename);
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
