use crate::component::preview::Preview;
use gloo_net::http::Request;
use log::info;
use yew::platform::*;
use yew::prelude::*;
use super::router::Route;
use yew_router::prelude::*;

pub struct Post {
    list: Vec<String>,
    preview: Vec<Html>,
    tags: Vec<(String, u16)>,
    selected_tag: String,
}

pub enum Msg {
    Update(Vec<String>),
    Tags(Vec<String>),
    TagSelect(String),
}

impl Component for Post {
    type Message = Msg;
    type Properties = ();

    fn create(_ctx: &Context<Self>) -> Self {
        Self {
            list: Vec::new(),
            preview: Vec::new(),
            tags: Vec::new(),
            selected_tag: String::new(),
        }
    }

    fn update(&mut self, _ctx: &Context<Self>, msg: Self::Message) -> bool {
        match msg {
            Msg::Update(list) => {
                self.list = list.clone();
                let preview: Vec<Html> = list
                    .iter()
                    .rev()
                    .map(|filename| {
                        let tag_cb = _ctx.link().callback(|tags: Vec<String>| Msg::Tags(tags));
                        html! { 
                            <Link<Route> classes={classes!("post-preview")} to={Route::Page { class: "post".to_string(), filename: filename.to_string() }}>
                                <Preview class={"post".to_string()} filename={filename.clone()} tag_cb={tag_cb.clone()} selected_tag={"".to_string()}/>
                            </Link<Route>>
                        }
                    })
                    .collect();
                self.preview = preview;
                true
            }
            Msg::Tags(tags) => {
                tags.iter().for_each(|tag| {
                    if let Some((_, value)) = self.tags.iter_mut().find(|(s, _)| s == tag) {
                        *value += 1;
                    } else {
                        self.tags.push((tag.to_string(), 1));
                    }
                });
                self.tags.sort_by_key(|k| std::cmp::Reverse(k.0.clone()));
                true
            }
            Msg::TagSelect(tag) => {
                if tag == self.selected_tag {
                    self.selected_tag = String::new();
                    let preview: Vec<Html> = self.list
                    .iter()
                    .rev()
                    .map(|filename| {
                        let tag_cb = _ctx.link().callback(|tags: Vec<String>| Msg::Tags(tags));
                        html! { 
                            <Link<Route> classes={classes!("post-preview")} to={Route::Page { class: "post".to_string(), filename: filename.to_string() }}>
                                <Preview class={"post".to_string()} filename={filename.clone()} tag_cb={tag_cb.clone()} selected_tag={"".to_string()}/>
                            </Link<Route>>
                        }
                    })
                    .collect();
                self.preview = preview;
                } else {
                    self.selected_tag = tag.clone();
                    let preview: Vec<Html> = self.list
                        .iter()
                        .rev()
                        .map(|filename| {
                            let tag_cb = _ctx.link().callback(|tags: Vec<String>| Msg::Tags(tags));
                            html! { 
                                <Link<Route> classes={classes!("post-preview")} to={Route::Page { class: "post".to_string(), filename: filename.to_string() }}>
                                    <Preview class={"post".to_string()} filename={filename.clone()} tag_cb={tag_cb.clone()} selected_tag={tag.clone()}/>
                                </Link<Route>>
                            }
                        })
                        .collect();
                    self.preview = preview;
                }
                true
            }
        }
    }

    fn view(&self, _ctx: &Context<Self>) -> Html {
        let preview = self.preview.clone();
        let tags: Vec<Html> = self
            .tags
            .iter()
            .rev()
            .map(|tag| {
                let tag_str = format!("#{} ({})", tag.0, tag.1);
                let tag_name = tag.0.clone();
                let class = if self.selected_tag == tag_name {
                    "post-tag-list selected"
                } else {
                    "post-tag-list"
                };
                html! {
                    <button class={classes!(class)}
                        onclick={_ctx.link().callback(move |_| Msg::TagSelect(tag_name.clone()))}>
                        { tag_str }
                    </button>
                }
            })
            .collect();

        html! {
            <div class="post-wrapper">
                <div class="post-container">
                    <div class="post-list">
                        { for preview }
                    </div>
                    if !tags.is_empty() {
                        <div class="post-tag">
                            { tags }
                        </div>
                    }
                </div>
            </div>
        }
    }

    fn rendered(&mut self, _ctx: &Context<Self>, first_render: bool) {
        if first_render {
            let link = _ctx.link().clone();
            spawn_local(async move {
                match Request::get("/post/list.json").send().await {
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
