use super::router::Route;
use yew::prelude::*;
use yew_router::prelude::*;

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
                    <Link<Route> classes={classes!("navbar-content")} to={Route::Home}>
                        { "RobotNotes" }
                        </Link<Route>>
                    <div class="navbar-content subtitle" style="justify-content: right;">
                        <Link<Route> classes={classes!("subtitle-content")} to={Route::Post}>
                        { "POST" }
                        </Link<Route>>
                        <div class="subtitle-content">
                            { "PORTFOLIO" }
                        </div>
                        <Link<Route> classes={classes!("subtitle-content")} to={Route::Archive}>
                            { "ARCHIVE" }
                        </Link<Route>>
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
