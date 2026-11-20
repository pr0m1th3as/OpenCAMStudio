//! A transparent text-input wrapper that reports focus/caret changes and owns
//! formula-specific Enter/completion keys. Normal text editing stays with iced.
use super::*;
use iced::advanced::{layout, mouse, renderer, widget, Clipboard, Layout, Shell, Widget};
use iced::{Event, Rectangle, Size};

type InputState =
    iced::widget::text_input::State<<iced::Renderer as iced::advanced::text::Renderer>::Paragraph>;

#[allow(clippy::too_many_arguments)]
pub(super) fn formula_input<'a>(
    inner: Element<'a, Message>,
    field: Field,
    source: &str,
    displayed: &str,
    editing: bool,
    cursor: usize,
    suggestions: Vec<&'static str>,
    active: usize,
) -> Element<'a, Message> {
    Element::new(FormulaInput {
        inner,
        field,
        source: source.into(),
        displayed: displayed.into(),
        editing,
        cursor,
        suggestions,
        active,
    })
}
struct FormulaInput<'a> {
    inner: Element<'a, Message>,
    field: Field,
    source: String,
    displayed: String,
    editing: bool,
    cursor: usize,
    suggestions: Vec<&'static str>,
    active: usize,
}
impl Widget<Message, iced::Theme, iced::Renderer> for FormulaInput<'_> {
    fn size(&self) -> Size<Length> {
        self.inner.as_widget().size()
    }
    fn tag(&self) -> widget::tree::Tag {
        self.inner.as_widget().tag()
    }
    fn state(&self) -> widget::tree::State {
        self.inner.as_widget().state()
    }
    fn children(&self) -> Vec<widget::Tree> {
        self.inner.as_widget().children()
    }
    fn diff(&self, tree: &mut widget::Tree) {
        self.inner.as_widget().diff(tree);
    }
    fn layout(
        &mut self,
        tree: &mut widget::Tree,
        renderer: &iced::Renderer,
        limits: &layout::Limits,
    ) -> layout::Node {
        self.inner.as_widget_mut().layout(tree, renderer, limits)
    }
    fn operate(
        &mut self,
        tree: &mut widget::Tree,
        layout: Layout<'_>,
        renderer: &iced::Renderer,
        operation: &mut dyn widget::Operation,
    ) {
        self.inner
            .as_widget_mut()
            .operate(tree, layout, renderer, operation);
    }
    fn draw(
        &self,
        tree: &widget::Tree,
        renderer: &mut iced::Renderer,
        theme: &iced::Theme,
        style: &renderer::Style,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        viewport: &Rectangle,
    ) {
        self.inner
            .as_widget()
            .draw(tree, renderer, theme, style, layout, cursor, viewport);
    }
    fn mouse_interaction(
        &self,
        tree: &widget::Tree,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        viewport: &Rectangle,
        renderer: &iced::Renderer,
    ) -> mouse::Interaction {
        self.inner
            .as_widget()
            .mouse_interaction(tree, layout, cursor, viewport, renderer)
    }
    fn update(
        &mut self,
        tree: &mut widget::Tree,
        event: &Event,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        renderer: &iced::Renderer,
        clipboard: &mut dyn Clipboard,
        shell: &mut Shell<'_, Message>,
        viewport: &Rectangle,
    ) {
        if matches!(
            event,
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right))
        ) && layout
            .bounds()
            .intersection(viewport)
            .is_some_and(|bounds| cursor.is_over(bounds))
        {
            shell.publish(Message::OpenParameterMenu(self.field));
            shell.capture_event();
            return;
        }
        let state = tree.state.downcast_mut::<InputState>();
        if state.is_focused() {
            if let Event::Keyboard(iced::keyboard::Event::KeyPressed {
                key: iced::keyboard::Key::Named(key),
                modifiers,
                ..
            }) = event
            {
                use iced::keyboard::key::Named;
                match key {
                    Named::Enter => {
                        state.unfocus();
                        shell.publish(Message::ParameterInputState(self.field, false, self.cursor));
                        shell.capture_event();
                        shell.request_redraw();
                        return;
                    }
                    Named::ArrowUp | Named::ArrowDown if !self.suggestions.is_empty() => {
                        shell.publish(Message::ParameterSuggestionMove(
                            self.field,
                            if *key == Named::ArrowUp { -1 } else { 1 },
                        ));
                        shell.capture_event();
                        return;
                    }
                    Named::Tab
                        if !modifiers.shift()
                            && !modifiers.control()
                            && !modifiers.alt()
                            && !modifiers.logo()
                            && !self.suggestions.is_empty() =>
                    {
                        shell.publish(Message::ParameterComplete(
                            self.field,
                            self.suggestions[self.active],
                            self.cursor,
                        ));
                        shell.capture_event();
                        return;
                    }
                    _ => {}
                }
            }
            if matches!(event, Event::Window(iced::window::Event::Unfocused)) {
                state.unfocus();
            }
        }
        let mut messages = Vec::new();
        let mut local = Shell::new(&mut messages);
        self.inner.as_widget_mut().update(
            tree, event, layout, cursor, renderer, clipboard, &mut local, viewport,
        );
        let changed_source = std::cell::RefCell::new(None);
        shell.merge(local, |message| {
            if let Message::FieldChanged(field, source) = &message {
                if *field == self.field {
                    *changed_source.borrow_mut() = Some(source.clone());
                }
            }
            message
        });
        let changed_source = changed_source.into_inner();
        let state = tree.state.downcast_mut::<InputState>();
        let focused = state.is_focused();
        if focused && !self.editing && changed_source.is_none() {
            state.select_all();
        }
        let current = changed_source.as_deref().unwrap_or(if self.editing {
            &self.source
        } else {
            &self.displayed
        });
        let value = iced::widget::text_input::Value::new(current);
        let position = match state.cursor().state(&value) {
            iced::widget::text_input::cursor::State::Index(index) => index,
            iced::widget::text_input::cursor::State::Selection { end, .. } => end,
        };
        if focused != self.editing || (focused && position != self.cursor) {
            shell.publish(Message::ParameterInputState(
                self.field,
                focused,
                if focused && !self.editing {
                    iced::widget::text_input::Value::new(&self.source).len()
                } else {
                    position
                },
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use iced::advanced::renderer::Headless;
    const FIELD: Field = Field::ContourRoughStepover;
    fn control(
        source: &str,
        editing: bool,
        caret: usize,
        suggestions: Vec<&'static str>,
        active: usize,
    ) -> Element<'static, Message> {
        let displayed = if editing { source } else { "0.600" };
        formula_input(
            inspector_input("", displayed)
                .on_input(|value| Message::FieldChanged(FIELD, value))
                .width(Length::Fixed(300.))
                .into(),
            FIELD,
            source,
            displayed,
            editing,
            caret,
            suggestions,
            active,
        )
    }
    fn key(key: iced::keyboard::Key, text: Option<&str>) -> Event {
        Event::Keyboard(iced::keyboard::Event::KeyPressed {
            modified_key: key.clone(),
            key,
            physical_key: iced::keyboard::key::Physical::Code(iced::keyboard::key::Code::KeyA),
            location: iced::keyboard::Location::Standard,
            modifiers: iced::keyboard::Modifiers::default(),
            text: text.map(Into::into),
            repeat: false,
        })
    }
    fn dispatch(
        element: &mut Element<'_, Message>,
        tree: &mut widget::Tree,
        renderer: &iced::Renderer,
        event: &Event,
        cursor: mouse::Cursor,
    ) -> Vec<Message> {
        let size = Size::new(320., 100.);
        let node =
            element
                .as_widget_mut()
                .layout(tree, renderer, &layout::Limits::new(Size::ZERO, size));
        let mut messages = Vec::new();
        element.as_widget_mut().update(
            tree,
            event,
            Layout::new(&node),
            cursor,
            renderer,
            &mut iced::advanced::clipboard::Null,
            &mut Shell::new(&mut messages),
            &Rectangle::with_size(size),
        );
        messages
    }
    #[test]
    #[ignore = "requires a graphics adapter; verifies real formula input focus and keyboard events"]
    fn formula_input_events() {
        let renderer = iced::futures::executor::block_on(<iced::Renderer as Headless>::new(
            iced::Font::DEFAULT,
            iced::Pixels(12.),
            None,
        ))
        .expect("graphics adapter");
        let click = Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left));
        let mut input = control("=6*0.1", false, 0, vec![], 0);
        let mut tree = widget::Tree::new(&input);
        let messages = dispatch(
            &mut input,
            &mut tree,
            &renderer,
            &click,
            mouse::Cursor::Available(iced::Point::new(20., 12.)),
        );
        assert!(messages
            .iter()
            .any(|m| matches!(m, Message::ParameterInputState(FIELD, true, 6))));
        input = control("=6*0.1", true, 6, vec![], 0);
        tree.diff(&input);
        let messages = dispatch(
            &mut input,
            &mut tree,
            &renderer,
            &key(
                iced::keyboard::Key::Named(iced::keyboard::key::Named::Enter),
                None,
            ),
            mouse::Cursor::Unavailable,
        );
        assert!(messages
            .iter()
            .any(|m| matches!(m, Message::ParameterInputState(FIELD, false, _))));
        assert!(!messages.iter().any(|m| matches!(m, Message::Apply)));
        assert!(!tree.state.downcast_ref::<InputState>().is_focused());
        // Moving focus elsewhere finishes editing too.
        tree.state.downcast_mut::<InputState>().focus();
        let messages = dispatch(
            &mut input,
            &mut tree,
            &renderer,
            &click,
            mouse::Cursor::Available(iced::Point::new(400., 80.)),
        );
        assert!(messages
            .iter()
            .any(|m| matches!(m, Message::ParameterInputState(FIELD, false, _))));
        // Tab inserts the highlighted variable instead of traversing focus.
        input = control(
            "=tool_",
            true,
            6,
            vec!["tool_diameter", "tool_cornerRadius"],
            1,
        );
        tree.diff(&input);
        tree.state.downcast_mut::<InputState>().focus();
        let messages = dispatch(
            &mut input,
            &mut tree,
            &renderer,
            &key(
                iced::keyboard::Key::Named(iced::keyboard::key::Named::Tab),
                None,
            ),
            mouse::Cursor::Unavailable,
        );
        assert!(messages
            .iter()
            .any(|m| matches!(m, Message::ParameterComplete(FIELD, "tool_cornerRadius", 6))));
        // New text and caret notifications must use the new length, not the old buffer.
        input = control("=tool_d", true, 7, vec![], 0);
        tree.diff(&input);
        tree.state.downcast_mut::<InputState>().focus();
        let messages = dispatch(
            &mut input,
            &mut tree,
            &renderer,
            &key(iced::keyboard::Key::Character("i".into()), Some("i")),
            mouse::Cursor::Unavailable,
        );
        assert!(messages
            .iter()
            .any(|m| matches!(m, Message::FieldChanged(FIELD, value) if value == "=tool_di")));
        assert!(messages
            .iter()
            .any(|m| matches!(m, Message::ParameterInputState(FIELD, true, 8))));
        let messages = dispatch(
            &mut input,
            &mut tree,
            &renderer,
            &Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right)),
            mouse::Cursor::Available(iced::Point::new(20., 12.)),
        );
        assert!(matches!(
            messages.as_slice(),
            [Message::OpenParameterMenu(FIELD)]
        ));
    }
}
