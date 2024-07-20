use yew::prelude::*;

pub struct Navbar;

impl Component for Navbar {
    type Message = ();
    type Properties = ();

    fn create(_ctx: &Context<Self>) -> Self {
        Self
    }

    fn view(&self, _ctx: &Context<Self>) -> Html {
        html! {
            <div class="navbar-container">
                <div class="navbar-content icon" style="justify-content: left;">
                    <i class="fa-solid fa-bars"></i>
                </div>
                <div class="navbar-content" style="justify-content: center;">
                    { "RobotNotes" }
                </div>
                <div class="navbar-content subtitle" style="justify-content: left;">
                    <div class="subtitle-content">
                        { "Topic 1" }
                    </div>
                    <div class="subtitle-content">
                        { "Topic 2" }
                    </div>
                </div>
                <div class="navbar-content icon" style="justify-content: right;">
                    <i class="fa-solid fa-magnifying-glass"></i>
                </div>
            </div>
        }
    }
}
