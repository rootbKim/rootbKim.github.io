use crate::component::metadata::extract_yaml_block_and_rest;
use gloo_net::http::Request;
use log::info;
use pulldown_cmark::{html, Options, Parser};
use wasm_bindgen::prelude::*;
use web_sys::window;
use yew::platform::*;
use yew::prelude::*;
use yew::virtual_dom::VNode;

pub struct About {
    text: String,
}

pub enum Msg {
    Update(String),
}

impl Component for About {
    type Message = Msg;
    type Properties = ();

    fn create(_ctx: &Context<Self>) -> Self {
        Self {
            text: String::new(),
        }
    }

    fn update(&mut self, _ctx: &Context<Self>, msg: Self::Message) -> bool {
        match msg {
            Msg::Update(text) => {
                let (_, rest_of_text) = extract_yaml_block_and_rest(&text);
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
        let html_content = self.markdown_to_html(&self.text);
        let vnodes = VNode::from_html_unchecked(html_content.into());
        html! {
            <>
                <div class="about-content">
                    { vnodes }
                </div>
            </>
        }
    }

    fn rendered(&mut self, _ctx: &Context<Self>, first_render: bool) {
        if first_render {
            let link = _ctx.link().clone();
            let url = format!("/about/about.md");
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

impl About {
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
