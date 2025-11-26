use std::time;

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct ExecutionProfile {
    pub(super) name: String,
    pub(super) begin: i64, // micro
    pub(super) end: i64,   // micro
    pub(super) depth: usize,
    pub(super) duration: time::Duration,
}

#[derive(Debug, Clone)]
pub struct ExecutionNode {
    pub(super) name: String,
    pub(super) begin: i64,
    pub(super) end: i64,
    pub(super) depth: usize,
    pub(super) duration: time::Duration,
    pub(super) next: Option<Box<ExecutionNode>>,
    pub(super) subdepth_child: Option<Box<ExecutionNode>>,
}

impl ExecutionNode {
    pub fn from(name: String, begin: i64, depth: usize) -> Self {
        ExecutionNode {
            name,
            begin,
            end: begin,
            depth,
            duration: time::Duration::from_secs(0),
            next: None,
            subdepth_child: None,
        }
    }
}

#[derive(Debug)]
pub struct ExecutionTree {
    top_time_nodes: Vec<ExecutionNode>,
    keep_last: bool,
}

impl ExecutionTree {
    pub fn new() -> Self {
        ExecutionTree {
            top_time_nodes: Vec::new(),
            keep_last: true,
        }
    }

    pub fn add(&mut self, name: String, time_int: i64, coordinates: Vec<usize>) {
        let depth = coordinates.len();

        // Get the right timed node:
        let current = self
            .top_time_nodes
            .iter_mut()
            .find(|node| node.begin == time_int);

        if let Some(mut current) = current {
            if coordinates[0] == 0 {
                if self.keep_last {
                    *current = ExecutionNode::from(name, time_int, depth);
                    return;
                } else {
                    panic!("Keep_last = false not implemented yet for Execution Tree");
                }
            }
            for coord in coordinates.iter().take(depth - 1) {
                for _j in 0..*coord {
                    current = current.next.as_mut().unwrap();
                }
                if current.subdepth_child.is_none() {
                    let time_int = current.begin;
                    current.subdepth_child =
                        Some(Box::new(ExecutionNode::from(name.clone(), time_int, depth)));
                    return;
                } else {
                    current = current.subdepth_child.as_mut().unwrap();
                }
            }
            for _j in 0..coordinates[depth - 1] - 1 {
                current = current.next.as_mut().unwrap();
            }
            let time_int = current.end;
            current.next = Some(Box::new(ExecutionNode::from(name, time_int, depth)));
        } else {
            self.top_time_nodes
                .push(ExecutionNode::from(name, time_int, depth));
        }
    }

    pub fn get_node(
        &mut self,
        _name: String,
        time_int: i64,
        coordinates: Vec<usize>,
    ) -> Option<&mut ExecutionNode> {
        let depth = coordinates.len();

        // Get the right timed node:
        let current = self
            .top_time_nodes
            .iter_mut()
            .find(|node| node.begin == time_int);

        if let Some(mut current) = current {
            if depth >= 2 {
                for coord in coordinates.iter().take(depth - 1) {
                    for _j in 0..*coord {
                        current = current.next.as_mut().unwrap();
                    }
                    if current.subdepth_child.is_none() {
                        return None;
                    } else {
                        current = current.subdepth_child.as_mut().unwrap();
                    }
                }
            }
            if *coordinates.last().unwrap() == 0 {
                return Some(current);
            }
            for _j in 0..*coordinates.last().unwrap() {
                current = current.next.as_mut().unwrap();
            }
            Some(current)
        } else {
            None
        }
    }

    fn recusive_iter(node: &ExecutionNode) -> Vec<ExecutionNode> {
        let mut nodes = Vec::new();
        let mut current = Some(node.clone());
        while current.is_some() {
            nodes.push(current.clone().unwrap());
            if let Some(child) = current.clone().unwrap().subdepth_child.as_deref() {
                nodes.append(&mut ExecutionTree::recusive_iter(child));
            }
            current = current.unwrap().next.as_deref().cloned();
        }
        nodes
    }

    pub fn iter<'a>(&'a self) -> impl Iterator<Item = ExecutionNode> + 'a {
        self.top_time_nodes
            .iter()
            .flat_map(move |node| ExecutionTree::recusive_iter(node).into_iter())
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    pub fn execution_tree_add() {
        let mut exe_tree = ExecutionTree::new();
        exe_tree.add("test0".to_string(), 1, vec![0]);
        assert!(exe_tree.top_time_nodes.len() == 1);
        assert!(exe_tree.top_time_nodes[0].name == "test0");
        assert!(exe_tree.top_time_nodes[0].next.is_none());
        assert!(exe_tree.top_time_nodes[0].subdepth_child.is_none());
        println!("{:?}", exe_tree);
        exe_tree.add("test1".to_string(), 1, vec![1]);
        assert!(exe_tree.top_time_nodes.len() == 1);
        assert!(exe_tree.top_time_nodes[0].next.as_ref().unwrap().name == "test1");
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .next
            .is_none());
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .is_none());
        println!("{:?}", exe_tree);
        exe_tree.add("test2".to_string(), 1, vec![1, 0]);
        assert!(exe_tree.top_time_nodes.len() == 1);
        assert!(exe_tree.top_time_nodes[0].next.as_ref().unwrap().name == "test1");
        assert!(
            exe_tree.top_time_nodes[0]
                .next
                .as_ref()
                .unwrap()
                .subdepth_child
                .as_ref()
                .unwrap()
                .name
                == "test2"
        );
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .as_ref()
            .unwrap()
            .next
            .is_none());
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .as_ref()
            .unwrap()
            .subdepth_child
            .is_none());
        println!("{:?}", exe_tree);
        exe_tree.add("test3".to_string(), 1, vec![1, 1]);
        assert!(exe_tree.top_time_nodes.len() == 1);
        assert!(exe_tree.top_time_nodes[0].next.as_ref().unwrap().name == "test1");
        assert!(
            exe_tree.top_time_nodes[0]
                .next
                .as_ref()
                .unwrap()
                .subdepth_child
                .as_ref()
                .unwrap()
                .name
                == "test2"
        );
        assert!(
            exe_tree.top_time_nodes[0]
                .next
                .as_ref()
                .unwrap()
                .subdepth_child
                .as_ref()
                .unwrap()
                .next
                .as_ref()
                .unwrap()
                .name
                == "test3"
        );
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .as_ref()
            .unwrap()
            .next
            .as_ref()
            .unwrap()
            .next
            .is_none());
        assert!(exe_tree.top_time_nodes[0]
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .as_ref()
            .unwrap()
            .next
            .as_ref()
            .unwrap()
            .subdepth_child
            .is_none());
        println!("{:?}", exe_tree);
    }

    #[test]
    pub fn execution_tree_get() {
        let mut exe_tree = ExecutionTree::new();
        exe_tree.add("test0".to_string(), 1, vec![0]);
        exe_tree.add("test1".to_string(), 1, vec![1]);
        exe_tree.add("test2".to_string(), 1, vec![1, 0]);
        exe_tree.add("test3".to_string(), 1, vec![1, 1]);
        let node = exe_tree.get_node("test0".to_string(), 1, vec![0]);
        assert!(node.is_some());
        let name = &node.unwrap().name;
        assert!(name == "test0", "Name: {:?} instead of test0", name);
        let node = exe_tree.get_node("test1".to_string(), 1, vec![1]);
        assert!(node.is_some());
        let name = &node.unwrap().name;
        assert!(name == "test1", "Name: {:?} instead of test1", name);
        let node = exe_tree.get_node("test2".to_string(), 1, vec![1, 0]);
        assert!(node.is_some());
        let name = &node.unwrap().name;
        assert!(name == "test2", "Name: {:?} instead of test2", name);
        let node = exe_tree.get_node("test3".to_string(), 1, vec![1, 1]);
        assert!(node.is_some());
        let name = &node.unwrap().name;
        assert!(name == "test3", "Name: {:?} instead of test3", name);
    }
}
