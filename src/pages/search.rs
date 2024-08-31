use super::router::Route;
use crate::component::metadata::{extract_yaml_block_and_rest, parse_yaml, Metadata};
use crate::component::preview::Preview;
use gloo_net::http::Request;
use log::info;
use web_sys::HtmlInputElement;
use yew::platform::*;
use yew::prelude::*;
use yew_router::prelude::*;

pub enum Msg {
    Update(String, String),
    PostList(Vec<String>),
    ArchiveList(Vec<String>),
    PortfolioList(Vec<String>),
    Input(String),
    Search,
}

pub struct Search {
    post_list: Vec<String>,
    archive_list: Vec<String>,
    portfolio_list: Vec<String>,
    input: String,
    search: bool,
    search_list: Vec<(String, String)>,
    preview: Vec<Html>,
    input_ref: NodeRef,
}

impl Component for Search {
    type Message = Msg;
    type Properties = ();

    fn create(_ctx: &Context<Self>) -> Self {
        Self {
            post_list: Vec::new(),
            archive_list: Vec::new(),
            portfolio_list: Vec::new(),
            input: String::new(),
            search: false,
            search_list: Vec::new(),
            preview: Vec::new(),
            input_ref: NodeRef::default(),
        }
    }

    fn update(&mut self, _ctx: &Context<Self>, msg: Self::Message) -> bool {
        match msg {
            Msg::Update(class, title) => {
                self.search_list.push((class, title));
                self.preview.clear();
                let preview: Vec<Html> = self.search_list
                    .iter()
                    .rev()
                    .map(|search| {
                        let class = search.0.clone();
                        let title = search.1.clone();
                        html! { 
                            <Link<Route> classes={classes!("search-preview")} to={Route::Page { class: class.clone(), filename: title.to_string() }}>
                                <Preview class={class.clone()} filename={title.clone()} tag_cb={None::<yew::Callback<Vec<String>>>} selected_tag={"".to_string()}/>
                            </Link<Route>>
                        }
                    })
                    .collect();
                self.preview = preview;
                true
            }
            Msg::PostList(list) => {
                self.post_list = list.clone();
                false
            }
            Msg::ArchiveList(list) => {
                self.archive_list = list.clone();
                false
            }
            Msg::PortfolioList(list) => {
                self.portfolio_list = list.clone();
                false
            }
            Msg::Input(input) => {
                self.input = input;
                false
            }
            Msg::Search => {
                if !self.input.is_empty() {
                    info!("{}", self.input);
                    self.search = true;
                    self.preview.clear();
                    self.search_list.clear();

                    let cb = _ctx
                        .link()
                        .callback(|msg: (String, String)| Msg::Update(msg.0, msg.1));
                    self.search_contents(cb.clone());
                }

                true
            }
        }
    }
    fn view(&self, _ctx: &Context<Self>) -> Html {
        let preview = self.preview.clone();
        let oninput = _ctx.link().callback(|e: InputEvent| {
            let input = e.target_unchecked_into::<HtmlInputElement>();
            Msg::Input(input.value())
        });
        let onkeydown = _ctx.link().callback(|e: KeyboardEvent| {
            if e.key() == "Enter" {
                Msg::Search
            } else {
                Msg::Input(e.target_unchecked_into::<HtmlInputElement>().value())
            }
        });
        let onclick = _ctx.link().callback(|_| Msg::Search);
        html! {
            <div class="search-wrapper">
                <div class="search-container">
                    <div class="search-title">
                        { "Search Your Notes" }
                    </div>
                    <div class="search-input-outer">
                        <div class="search-input-inner">
                            <input
                                ref={self.input_ref.clone()}
                                class="search-input"
                                placeholder="Try to search your topics"
                                {oninput}
                                {onkeydown}
                            />
                            <button class="search-button" {onclick}>
                                { "SEARCH" }
                            </button>
                        </div>
                    </div>
                    if self.search {
                        <div class="search-list">
                            { for preview }
                        </div>
                    }
                </div>
            </div>
        }
    }

    fn rendered(&mut self, _ctx: &Context<Self>, first_render: bool) {
        if first_render {
            if let Some(input) = self.input_ref.cast::<HtmlInputElement>() {
                input.focus().unwrap();
            }

            let link = _ctx.link().clone();
            spawn_local(async move {
                match Request::get("/post/list.json").send().await {
                    Ok(resp) => match resp.text().await {
                        Ok(text) => match serde_json::from_str::<Vec<String>>(&text) {
                            Ok(list) => {
                                link.send_message(Msg::PostList(list));
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

                match Request::get("/archive/list.json").send().await {
                    Ok(resp) => match resp.text().await {
                        Ok(text) => match serde_json::from_str::<Vec<String>>(&text) {
                            Ok(list) => {
                                link.send_message(Msg::ArchiveList(list));
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

                match Request::get("/portfolio/list.json").send().await {
                    Ok(resp) => match resp.text().await {
                        Ok(text) => match serde_json::from_str::<Vec<String>>(&text) {
                            Ok(list) => {
                                link.send_message(Msg::PortfolioList(list));
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

impl Search {
    pub fn search_contents(
        &self,
        update: Callback<(String, String)>,
    ) {
        let class_list: Vec<String> = vec!["archive".to_string(), "post".to_string(), "portfolio".to_string()];
        class_list.iter().for_each(|class| {
            let mut list: Vec<String> = Vec::new();
            if class == "post" {
                list = self.post_list.clone();
            } else if class == "archive" {
                list = self.archive_list.clone();
            } else if class == "portfolio" {
                list = self.portfolio_list.clone();
            }
            list.iter().for_each(|title| {
                let update: Callback<(String, String)> = update.clone();
                let class = class.clone();
                let url = format!("/{}/{}.md", class, title);
                let input = self.input.clone().to_lowercase();
                let title = title.clone();
                spawn_local(async move {
                    match Request::get(url.as_str()).send().await {
                        Ok(resp) => match resp.text().await {
                            Ok(text) => {
                                let (yaml_block, rest_of_text) = extract_yaml_block_and_rest(&text);

                                let mut metadata: Metadata = Metadata::default();
                                let mut text: String = String::new();

                                let mut find: bool = false;
                                match yaml_block {
                                    Some(block) => match parse_yaml(block) {
                                        Ok(m) => {
                                            metadata = m.clone();
                                            if metadata.title.unwrap_or_default().to_lowercase().contains(&input)
                                                || metadata
                                                    .excerpt
                                                    .unwrap_or_default()
                                                    .to_lowercase()
                                                    .contains(&input)
                                            {
                                                update.emit((class.clone(), title.clone()));
                                                find = true;
                                            }

                                            if !find {
                                                for tag in metadata.tags.unwrap_or_default() {
                                                    if tag.to_lowercase().contains(&input) {
                                                        update.emit((class.clone(), title.clone()));
                                                        find = true;
                                                    }
                                                }
                                            }
                                        }
                                        Err(e) => info!("Error parsing YAML: {:?}", e),
                                    },
                                    None => info!("No YAML block found."),
                                }

                                if !find {
                                    match rest_of_text {
                                        Some(rest) => {
                                            text = rest.to_string();
                                            if text.to_lowercase().contains(&input) {
                                                update.emit((class.clone(), title.clone()));
                                            }
                                        }
                                        None => info!("No rest of the text found."),
                                    }
                                }
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
            });
        });
    }
}
