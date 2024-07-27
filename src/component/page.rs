use crate::component::metadata::{extract_yaml_block_and_rest, parse_yaml, Metadata};
use gloo_net::http::Request;
use log::info;
use pulldown_cmark::{html, Options, Parser};
use yew::platform::*;
use yew::prelude::*;
use yew::virtual_dom::VNode;

pub struct Page {
    metadata: Metadata,
    text: String,
}

#[derive(Properties, PartialEq)]
pub struct Props {
    pub filename: String,
}

pub enum Msg {
    Update(String),
}

impl Component for Page {
    type Message = Msg;
    type Properties = Props;

    fn create(_ctx: &Context<Self>) -> Self {
        Self {
            metadata: Metadata::default(),
            text: String::new(),
        }
    }

    fn update(&mut self, _ctx: &Context<Self>, msg: Self::Message) -> bool {
        match msg {
            Msg::Update(text) => {
                let (yaml_block, rest_of_text) = extract_yaml_block_and_rest(&text);

                match yaml_block {
                    Some(block) => match parse_yaml(block) {
                        Ok(metadata) => {
                            self.metadata = metadata;
                        }
                        Err(e) => info!("Error parsing YAML: {:?}", e),
                    },
                    None => info!("No YAML block found."),
                }

                match rest_of_text {
                    Some(rest) => {
                        self.text = rest.to_string();
                    }
                    None => info!("No rest of the text found."),
                }
                true
            }
        }
    }

    fn view(&self, _ctx: &Context<Self>) -> Html {
        let title = self.metadata.title.clone().unwrap_or_default();
        let date = self.metadata.date.clone().unwrap_or_default();
        let excerpt = self.metadata.excerpt.clone().unwrap_or_default();

        let html_content = self.markdown_to_html(&self.text);
        let vnodes = VNode::from_html_unchecked(html_content.into());

        html! {
            <>
                <div class="page-metadata">
                    <div class="page-title">
                        { title }
                    </div>
                    <div class="page-date">
                        { date }
                    </div>
                    <div class="page-excerpt">
                        { excerpt }
                    </div>
                </div>
                <div class="page-content">
                    { vnodes }
                </div>
            </>
        }
    }

    fn rendered(&mut self, _ctx: &Context<Self>, first_render: bool) {
        if first_render {
            let link = _ctx.link().clone();
            let url = format!("/markdown/{}.md", _ctx.props().filename);
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

impl Page {
    pub fn markdown_to_html(&self, markdown: &str) -> String {
        let mut options = Options::empty();
        options.insert(Options::ENABLE_TABLES);
        options.insert(Options::ENABLE_FOOTNOTES);
        options.insert(Options::ENABLE_STRIKETHROUGH);
        options.insert(Options::ENABLE_TASKLISTS);

        let parser = Parser::new_ext(markdown, options);
        let mut html_output = String::new();
        html::push_html(&mut html_output, parser);
        html_output
    }
}
