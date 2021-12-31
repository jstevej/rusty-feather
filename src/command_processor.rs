use heapless::{String, Vec};

use crate::console::{ack, error, info};

pub const MAX_TOKENS: usize = 4;

#[derive(PartialEq, Eq)]
pub enum CommandResult {
    Error(&'static str),
    Handled,
    InvalidArguments,
    NotHandled,
}

pub struct CommandProcessor<const MSG_SIZE: usize> {
    resp: String<MSG_SIZE>,
}

impl<const MSG_SIZE: usize> CommandProcessor<MSG_SIZE> {
    pub fn new() -> CommandProcessor<MSG_SIZE> {
        let resp: String<MSG_SIZE> = String::new();
        Self { resp }
    }

    fn assemble(&mut self, tokens: &Vec<&str, MAX_TOKENS>) -> &str {
        self.resp.clear();

        for t in tokens.iter().take(tokens.len() - 1) {
            let _ = self.resp.push_str(t);
            let _ = self.resp.push(' ');
        }

        let _ = self.resp.push_str(tokens[tokens.len() - 1]);
        self.resp.as_str()
    }

    pub fn handle_result(&mut self, tokens: &Vec<&str, MAX_TOKENS>, result: CommandResult) -> bool {
        self.resp.clear();

        match result {
            CommandResult::Error(msg) => {
                let _ = self.resp.push_str(tokens[0]);
                let _ = self.resp.push_str(": ");
                let _ = self.resp.push_str(msg);
                error(self.resp.as_str());
            },
            CommandResult::Handled => {
                ack(tokens[0]);
            },
            CommandResult::InvalidArguments => {
                let _ = self.resp.push_str(tokens[0]);
                let _ = self.resp.push_str(": invalid arguments");
                error(self.resp.as_str());
            },
            CommandResult::NotHandled => {
                return false;
            },
        }

        return true;
    }

    pub fn process(
        &mut self,
        tokens: &Vec<&str, MAX_TOKENS>
    ) {
        if tokens.len() > 0 {
            if tokens[0] == "echo" {
                info(self.assemble(&tokens));
            } else if tokens[0] == "panic" {
                let x = [0, 1, 2];
                let i = x.len() + 1;
                let _y = x[i];
            } else {
                self.resp.clear();
                let _ = self.resp.push_str(tokens[0]);
                let _ = self.resp.push_str(": unknown command");
                error(self.resp.as_str());
                return;
            }

            ack(tokens[0]);
        }
    }

    pub fn tokenize<'a>(&self, cmd: &'a String<MSG_SIZE>) -> Vec<&'a str, MAX_TOKENS> {
        let mut tokens: Vec<&str, MAX_TOKENS> = Vec::new();

        for token in cmd.split_ascii_whitespace() {
            let _ = tokens.push(token);
        }

        tokens
    }
}
