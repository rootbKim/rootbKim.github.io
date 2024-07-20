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
            <div class="navbar-wrapper">
                <div class="navbar-container">
                    <div class="navbar-content" style="justify-content: center;">
                        { "RobotNotes" }
                    </div>
                    <div class="navbar-content subtitle" style="justify-content: right;">
                        <div class="subtitle-content">
                            { "BLOG" }
                        </div>
                        <div class="subtitle-content">
                            { "PORTFOLIO" }
                        </div>
                        <div class="subtitle-content">
                            { "ARCHIVE" }
                        </div>
                        <div class="subtitle-content">
                            { "ABOUT" }
                        </div>
                    </div>
                    <div class="navbar-content icon" style="justify-content: right;">
                        <i class="fa-solid fa-magnifying-glass"></i>
                    </div>
                </div>
            </div>
        }
    }
}
