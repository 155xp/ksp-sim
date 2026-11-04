//! Minimal KSP `ConfigNode` text parser (the `.cfg` format of parts, Physics.cfg, PartDatabase.cfg).
//!
//! Format (ConfigNode.Parse / ConfigNode.Load): `name = value` lines, `NAME { ... }` nodes (the
//! brace may be on the next line), `//` comments. Values keep everything after the first `=`,
//! trimmed. Used to load stock parts directly from the user's KSP install at runtime so no game
//! data is redistributed with the sim.

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Node {
    pub name: String,
    pub values: Vec<(String, String)>,
    pub nodes: Vec<Node>,
}

impl Node {
    pub fn get(&self, key: &str) -> Option<&str> {
        self.values.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }
    pub fn get_all<'a>(&'a self, key: &'a str) -> impl Iterator<Item = &'a str> + 'a {
        self.values.iter().filter(move |(k, _)| k == key).map(|(_, v)| v.as_str())
    }
    pub fn get_f64(&self, key: &str) -> Option<f64> {
        self.get(key).and_then(|v| v.trim().parse().ok())
    }
    pub fn node(&self, name: &str) -> Option<&Node> {
        self.nodes.iter().find(|n| n.name == name)
    }
    pub fn nodes_named<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Node> + 'a {
        self.nodes.iter().filter(move |n| n.name == name)
    }
}

/// Parse a whole file into an unnamed root node.
pub fn parse(text: &str) -> Node {
    let mut stack: Vec<Node> = vec![Node::default()];
    let mut pending: Option<String> = None;
    for raw in text.lines() {
        let line = match raw.find("//") {
            Some(i) => &raw[..i],
            None => raw,
        };
        let mut rest = line.trim().trim_start_matches('\u{feff}');
        while !rest.is_empty() {
            if let Some(r) = rest.strip_prefix('{') {
                let name = pending.take().unwrap_or_default();
                stack.push(Node { name, ..Default::default() });
                rest = r.trim_start();
            } else if let Some(r) = rest.strip_prefix('}') {
                if let Some(p) = pending.take() {
                    stack.last_mut().unwrap().values.push((p, String::new()));
                }
                if stack.len() > 1 {
                    let n = stack.pop().unwrap();
                    stack.last_mut().unwrap().nodes.push(n);
                }
                rest = r.trim_start();
            } else {
                // a token up to the next brace
                let end = rest.find(['{', '}']).unwrap_or(rest.len());
                let tok = rest[..end].trim();
                if let Some(eq) = tok.find('=') {
                    if let Some(p) = pending.take() {
                        stack.last_mut().unwrap().values.push((p, String::new()));
                    }
                    let (k, v) = (tok[..eq].trim(), tok[eq + 1..].trim());
                    stack.last_mut().unwrap().values.push((k.to_string(), v.to_string()));
                } else if !tok.is_empty() {
                    pending = Some(tok.to_string());
                }
                rest = rest[end..].trim_start();
            }
        }
    }
    while stack.len() > 1 {
        let n = stack.pop().unwrap();
        stack.last_mut().unwrap().nodes.push(n);
    }
    stack.pop().unwrap()
}

/// Parse a `FloatCurve` node (`key = t v [in out]` lines).
pub fn curve(node: &Node) -> crate::curve::Curve {
    crate::curve::Curve::from_cfg_lines(node.get_all("key"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_nested_nodes_and_comments() {
        let t = "PART\n{\n name = foo // c\n MODULE\n {\n  name = ModuleEngines\n  atmosphereCurve\n  {\n   key = 0 345\n   key = 1 290\n  }\n }\n MODULE { name = X }\n}\n";
        let r = parse(t);
        let p = r.node("PART").unwrap();
        assert_eq!(p.get("name"), Some("foo"));
        let m: Vec<_> = p.nodes_named("MODULE").collect();
        assert_eq!(m.len(), 2);
        assert_eq!(m[1].get("name"), Some("X"));
        let c = curve(m[0].node("atmosphereCurve").unwrap());
        assert_eq!(c.keys.len(), 2);
        assert_eq!(c.eval(1.0), 290.0);
    }
}
