use yew::prelude::*;

pub struct Blog;
impl Component for Blog {
    type Message = ();
    type Properties = ();

    fn create(_ctx: &Context<Self>) -> Self {
        Self
    }

    fn view(&self, _ctx: &Context<Self>) -> Html {
        html! {
            <div class="post-wrapper">
                <div class="post-container">
                    <div class="post-list">
                        { "list" }
                    </div>
                    <div class="post-tag">
                        { "tag" }
                    </div>
                </div>
            </div>
        }
    }
}