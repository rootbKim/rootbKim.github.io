use crate::component::metadata::{extract_yaml_block_and_rest, parse_yaml, Metadata};
use gloo_net::http::Request;
use log::info;
use pulldown_cmark::{html, Options, Parser};
use wasm_bindgen::prelude::*;
use web_sys::window;
use yew::platform::*;
use yew::prelude::*;
use yew::virtual_dom::VNode;

pub struct Page {
    metadata: Metadata,
    text: String,
    not_found: bool,
}

#[derive(Properties, PartialEq)]
pub struct Props {
    pub class: String,
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
            not_found: false,
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
                        Err(e) => self.not_found = true,
                    },
                    None => self.not_found = true,
                }

                match rest_of_text {
                    Some(rest) => {
                        self.text = rest.to_string();
                    }
                    None => self.not_found = true,
                }
                true
            }
        }
    }

    fn view(&self, _ctx: &Context<Self>) -> Html {
        if self.not_found {
            html! {
                <>
                    <div class="not-found">
                        {"Page Not Found"}
                    </div>
                    <div class="not-found-contents">
                        { "The page you visited has an invalid or deleted address." }
                    </div>
                </>
            }
        } else {
            let title = self.metadata.title.clone().unwrap_or_default();
            let date = self.metadata.date.clone().unwrap_or_default();
            let excerpt = self.metadata.excerpt.clone().unwrap_or_default();

            let html_content = self.markdown_to_html(&self.text);
            let vnodes = VNode::from_html_unchecked(html_content.into());

            let tags: Vec<Html> = self
                .metadata
                .tags
                .as_ref()
                .unwrap_or(&mut Vec::new())
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
                        <div class="page-tags">
                            { tags }
                        </div>
                    </div>
                    <div class="page-content">
                        { vnodes }
                    </div>
                </>
            }
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

        if let Some(win) = window() {
            let func = win
                .get("eval")
                .unwrap()
                .dyn_into::<js_sys::Function>()
                .unwrap();

            let js_code = "Prism.highlightAll()";

            let _result = func.call1(&JsValue::NULL, &JsValue::from_str(js_code));
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
