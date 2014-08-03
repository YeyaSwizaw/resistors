use std::{vec, os};
use std::io::{File, BufferedWriter};
use std::fmt::{Show, Formatter, Result};
use std::collections::{TreeSet, HashMap, PriorityQueue};

#[deriving(Clone)]
enum ResistorTree {
    Resistor(String, Option<f64>),
    Parallel(Vec<ResistorTree>, Option<f64>),
    Series(Vec<ResistorTree>, Option<f64>)
}

impl ResistorTree {
    pub fn ohms(&self) -> f64 {
        match self {
            &Resistor(ref s, ref val) => {
                match val {
                    &Some(value) => value,
                    &None => {
                        match from_str(s.as_slice()) {
                            Some(val) => val,
                            None => fail!("Error converting resistor value")
                        }
                    }
                }
            },

            &Parallel(ref vec, ref val) => {
                match val {
                    &Some(value) => value,
                    &None => 1.0 / vec.iter().fold(0.0, |acc, rs| acc + (1.0 / rs.ohms()))
                }
            },

            &Series(ref vec, ref val) => {
                match val {
                    &Some(value) => value,
                    &None => 1.0 / vec.iter().fold(0.0, |acc, rs| acc + rs.ohms())
                }
            },

        }
    }

    pub fn add_in_series<'a>(&'a self, other: &'a ResistorTree) -> ResistorTree {
        match (self, other) {
            (&Series(ref v1, _), &Series(ref v2, _)) => Series(v1.add(v2), None),
            (&Series(ref v, _), t) | (t, &Series(ref v, _)) => Series(v.add(&vec!(t.clone())), None),
            (t1, t2) => Series(vec!(t1.clone(), t2.clone()), None)
        }
    }

    pub fn add_in_parallel<'a>(&'a self, other: &'a ResistorTree) -> ResistorTree {
        match (self, other) {
            (&Parallel(ref v1, _), &Parallel(ref v2, _)) => Parallel(v1.add(v2), None),
            (&Parallel(ref v, _), t) | (t, &Parallel(ref v, _)) => Parallel(v.add(&vec!(t.clone())), None),
            (t1, t2) => Parallel(vec!(t1.clone(), t2.clone()), None)
        }
    }

    pub fn total_resistors(&self) -> uint {
        match self {
            &Resistor(ref s, _) => 1,
            &Parallel(ref v, _) | &Series(ref v, _) => v.iter().fold(0, |acc, rt| acc + rt.total_resistors())
        }
    }

    pub fn count_resistors(&self) -> HashMap<String, uint> {
        let mut hm = HashMap::new();
        self.count(&mut hm);
        hm
    }

    fn count(&self, hm: &mut HashMap<String, uint>) {
        match self {
            &Resistor(ref value, _) => { hm.insert_or_update_with(value.clone(), 1, |_k, v| *v += 1); }
            &Parallel(ref vec, _) | &Series(ref vec, _) => { for item in vec.iter() { item.count(hm) }; }
        }
    }
}

impl Show for ResistorTree {
    fn fmt(&self, f: &mut Formatter) -> Result {
        match self {
            &Resistor(ref value, _) => try!(write!(f, "{}Ω", value)),

            &Parallel(ref vec, _) => {
                try!(write!(f, "Parallel["));

                let mut first = true;
                for rt in vec.iter() {
                    if first {
                        first = false;
                    } else {
                        try!(write!(f, ", "));
                    }
                    try!(write!(f, "{}", rt));
                }

                try!(write!(f, "]"));
            },

            &Series(ref vec, _) => {
                try!(write!(f, "Series["));

                let mut first = true;
                for rt in vec.iter() {
                    if first {
                        first = false;
                    } else {
                        try!(write!(f, ", "));
                    }
                    try!(write!(f, "{}", rt));
                }

                try!(write!(f, "]"));
            }
        }

        Ok(())
    }
}

impl PartialEq for ResistorTree {
    fn eq(&self, other: &ResistorTree) -> bool {
        match (self, other) {
            (&Resistor(ref val1, _), &Resistor(ref val2, _)) => *val1 == *val2,

            (&Parallel(ref v1, _), &Parallel(ref v2, _)) | (&Series(ref v1, _), &Series(ref v2, _)) => {
                for (t1, t2) in v1.iter().zip(v2.iter()) {
                    if(t1 != t2) {
                        return false;
                    }
                }

                true
            },

            _ => false
        }
    }

    fn ne(&self, other: &ResistorTree) -> bool {
        !(self == other)
    }
}

impl Eq for ResistorTree {}

impl PartialOrd for ResistorTree {
    fn partial_cmp(&self, other: &ResistorTree) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ResistorTree {
    fn cmp(&self, other: &ResistorTree) -> Ordering {
        match self.total_resistors().cmp(&other.total_resistors()) {
            Equal => if self.eq(other) { Equal } else if self.ohms() < other.ohms() { Less } else { Greater },
            Less => Greater,
            Greater => Less
        }
    }
}

struct ResistorSearcher<'a> {
    target: f64,
    available: &'a Vec<f64>,
    threshold: f64,

    queue: PriorityQueue<ResistorTree>,
    explored: TreeSet<ResistorTree>
}

impl<'a> ResistorSearcher<'a> {
    pub fn new(target: f64, resistors: &'a Vec<f64>, threshold: f64) -> ResistorSearcher<'a> {
        ResistorSearcher {
            target: target,
            available: resistors,
            threshold: threshold,

            queue: PriorityQueue::new(),
            explored: TreeSet::new()
        }
    }

    fn is_goal<'c>(&self, rt: &'c ResistorTree) -> bool{
        (rt.ohms() - self.target).abs() < self.threshold
    }

    fn explore_node<'c>(&mut self, rt: &'c ResistorTree) {
        match rt {
            &Resistor(_, _) => {
                for val in self.available.iter() {
                    let new_r = Resistor(val.to_string(), None);

                    self.push_node(rt.add_in_series(&new_r));
                    self.push_node(rt.add_in_parallel(&new_r));
                }
            },

            &Series(ref v, _) => {
                for val in self.available.iter() {
                    let new_r = Resistor(val.to_string(), None);
                    self.push_node(rt.add_in_series(&new_r));
                    self.push_node(rt.add_in_parallel(&new_r));

                    for i in range(0, v.len()) {
                        let mut new_v = vec::Vec::new();

                        for j in range(0, v.len()) {
                            if i == j {
                                new_v.push(v[j].clone());
                            } else {
                                new_v.push(v[j].add_in_parallel(&new_r));
                            }
                        }

                        self.push_node(Series(new_v, None));
                    }
                }
            },

            &Parallel(ref v, _) => {
                for val in self.available.iter() {
                    let new_r = Resistor(val.to_string(), None);
                    self.push_node(rt.add_in_series(&new_r));
                    self.push_node(rt.add_in_parallel(&new_r));

                    for i in range(0, v.len()) {
                        let mut new_v = vec::Vec::new();

                        for j in range(0, v.len()) {
                            if i == j {
                                new_v.push(v[j].clone());
                            } else {
                                new_v.push(v[j].add_in_series(&new_r));
                            }
                        }

                        self.push_node(Parallel(new_v, None));
                    }
                }
            }
        }
    }

    fn push_node(&mut self, rt: ResistorTree) {
        if !self.explored.contains(&rt) {
            self.explored.insert(rt.clone());
            self.queue.push(rt);
        }
    }

    fn search(&mut self) -> Option<ResistorTree> {
        for val in self.available.iter() {
            self.push_node(Resistor(val.to_string(), None));
        }

        while !self.queue.is_empty() {
            let rt = self.queue.pop().unwrap();

                if self.is_goal(&rt) {
                    return Some(rt.clone())
                }

                self.explore_node(&rt);
        }

        None
    }
}

fn print_usage() {
    println!("Usage: [OPTIONS] [available resistors]");
    println!("Options:");
    println!("    -r    resistance target");
    println!("    -t    resistance threshold (optional, default is 0.1)");
    println!("    -f    frequency target (takes priority over -r)");
}

fn resistor_search<'a>(target: f64, resistors: &'a Vec<f64>, threshold: f64) -> Option<ResistorTree> {
    println!("Initialising search for target resistance: {}Ω, threshold {}Ω", target, threshold);
    println!("");

    match ResistorSearcher::new(target, resistors, threshold).search() {
        Some(tree) => {
            println!("Arrangement found with resistance {}Ω: ", tree.ohms());
            println!("{}", tree);
            println!("");
            println!("Resistor count:");
            println!("{}", tree.count_resistors());

            Some(tree)
        },

        None => {
            println!("No arrangement found!");
            None
        }
    }
}

fn freq_search<'a>(target: f64, resistors: &'a Vec<f64>) {
    println!("To achieve a frequency of {}Hz:", target);

    let high = 0.75 / target;
    let low = 0.25 / target;
    let r1 = (high - low) / (0.6931 * 0.000047);
    let r2 = high / (0.6931 * 0.000047);

    println!("R1 = {}Ω", r1);
    println!("R2 = {}Ω", r2);
    println!("");

    let thresh_freq = target * 0.99;
    let thresh_high = 0.75 / thresh_freq;
    let thresh_low = 0.25 / thresh_freq;
    let thresh_1 = ((thresh_high - thresh_low) / (0.6931 * 0.000047)) - r1;
    let thresh_2 = (thresh_high / (0.6931 * 0.000047)) - r2;

    resistor_search(r1, resistors, thresh_1);
    resistor_search(r2, resistors, thresh_2);
}

fn main() {
    let args = os::args();

    let mut target: f64 = -1.0;
    let mut threshold: f64 = 0.1;
    let mut freq: Option<f64> = None;

    let mut i = 1;
    while i < args.iter().len() {
        if args[i].as_slice() == "-r" {
            if i + 1 >= args.iter().len() {
                return print_usage()
            }

            target = match from_str(args[i + 1].as_slice()) {
                Some(val) => val,
                None => return print_usage()
            };

            i += 2;
        } else if args[i].as_slice() == "-f" {
            if i + 1 >= args.iter().len() {
                return print_usage()
            }

            freq = match from_str(args[i + 1].as_slice()) {
                Some(val) => Some(val),
                None => return print_usage()
            };

            i += 2;
        } else {
            break;
        }
    }

    let mut resistors: Vec<f64> = vec::Vec::new();

    for value in args.tailn(i).iter() {
        match from_str(value.as_slice()) {
            Some(val) => resistors.push(val),
            None => return print_usage()
        };
    };

    match freq {
        None => {
            if target < 0.0 {
                return print_usage();
            } else {
                resistor_search(target, &resistors, target * 0.01); 
            }
        },

        Some(val) => {
            if val < 0.0 {
                return print_usage();
            } else {
                freq_search(val, &resistors);
            }
        }
    };
}

