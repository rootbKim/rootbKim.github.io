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
                        <Link<Route> classes={classes!("subtitle-content")} to={Route::Portfolio}>
                            { "PORTFOLIO" }
                        </Link<Route>>
                        <Link<Route> classes={classes!("subtitle-content")} to={Route::Archive}>
                            { "ARCHIVE" }
                        </Link<Route>>
                        <Link<Route> classes={classes!("subtitle-content")} to={Route::About}>
                            { "ABOUT" }
                        </Link<Route>>
                    </div>
                    <Link<Route> classes={classes!("navbar-content", "icon")} to={Route::Search}>
                        <i class="fa-solid fa-magnifying-glass"></i>
                    </Link<Route>>
                </div>
            </div>
        }
    }
}
