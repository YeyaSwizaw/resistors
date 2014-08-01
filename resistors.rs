use std::{vec, os};
use std::fmt::{Show, Formatter, Result};
use std::collections::{Deque, RingBuf, TreeSet};

#[deriving(Clone, PartialEq)]
enum ResistorTree {
    Resistor(f64),
    Parallel(Vec<ResistorTree>),
    Series(Vec<ResistorTree>)
}

impl ResistorTree {
    pub fn ohms(&self) -> f64 {
        match self {
            &Resistor(value) => value,
            &Parallel(ref vec) => 1.0 / vec.iter().fold(0.0, |acc, rs| acc + (1.0 / rs.ohms())),
            &Series(ref vec) => vec.iter().fold(0.0, |acc, rs| acc + rs.ohms())
        }
    }

    pub fn add_in_series<'a>(&'a self, other: &'a ResistorTree) -> ResistorTree {
        match (self, other) {
            (&Series(ref v1), &Series(ref v2)) => Series(v1.add(v2)),
            (&Series(ref v), t) | (t, &Series(ref v)) => Series(v.add(&vec!(t.clone()))),
            (t1, t2) => Series(vec!(t1.clone(), t2.clone()))
        }
    }

    pub fn add_in_parallel<'a>(&'a self, other: &'a ResistorTree) -> ResistorTree {
        match (self, other) {
            (&Parallel(ref v1), &Parallel(ref v2)) => Parallel(v1.add(v2)),
            (&Parallel(ref v), t) | (t, &Parallel(ref v)) => Parallel(v.add(&vec!(t.clone()))),
            (t1, t2) => Parallel(vec!(t1.clone(), t2.clone()))
        }
    }
}

impl Show for ResistorTree {
    fn fmt(&self, f: &mut Formatter) -> Result {
        match self {
            &Resistor(value) => try!(write!(f, "{}Ω", value)),

            &Parallel(ref vec) => {
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

            &Series(ref vec) => {
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

impl Eq for ResistorTree {}

impl PartialOrd for ResistorTree {
    fn partial_cmp(&self, other: &ResistorTree) -> Option<Ordering> {
        self.ohms().partial_cmp(&other.ohms())
    }
}

impl Ord for ResistorTree {
    fn cmp(&self, other: &ResistorTree) -> Ordering {
        match self.ohms().partial_cmp(&other.ohms()) {
            Some(o) => o,
            None => {
                if (self.ohms() - other.ohms()).abs() < 0.1 {
                    Equal
                } else if self.ohms() < other.ohms() {
                    Less
                } else {
                    Greater
                }
            }
        }
    }
}

struct ResistorSearcher<'a> {
    target: f64,
    available: &'a Vec<f64>,
    threshold: f64,

    queue: RingBuf<ResistorTree>,
    explored: TreeSet<ResistorTree>
}

impl<'a> ResistorSearcher<'a> {
    pub fn new(target: f64, resistors: &'a Vec<f64>, threshold: f64) -> ResistorSearcher<'a> {
        ResistorSearcher {
            target: target,
            available: resistors,
            threshold: threshold,

            queue: RingBuf::new(),
            explored: TreeSet::new()
        }
    }

    fn is_goal<'b>(&self, rt: &'b ResistorTree) -> bool{
        (rt.ohms() - self.target).abs() < self.threshold
    }

    fn explore_node<'b>(&mut self, rt: &'b ResistorTree) {
        match rt {
            &Resistor(_) => {
                for val in self.available.iter() {
                    let new_r = Resistor(*val);

                    self.queue.push(rt.add_in_series(&new_r));
                    self.queue.push(rt.add_in_parallel(&new_r));
                }
            },

            &Series(ref v) => {
                for val in self.available.iter() {
                    let new_r = Resistor(*val);
                    self.queue.push(rt.add_in_series(&new_r));
                    self.queue.push(rt.add_in_parallel(&new_r));

                    for i in range(0, v.len()) {
                        let mut new_v = vec::Vec::new();

                        for j in range(0, v.len()) {
                            if i == j {
                                new_v.push(v[j].clone());
                            } else {
                                new_v.push(v[j].add_in_parallel(&new_r));
                            }
                        }

                        self.queue.push(Series(new_v));
                    }
                }
            },

            &Parallel(ref v) => {
                for val in self.available.iter() {
                    let new_r = Resistor(*val);
                    self.queue.push(rt.add_in_series(&new_r));
                    self.queue.push(rt.add_in_parallel(&new_r));

                    for i in range(0, v.len()) {
                        let mut new_v = vec::Vec::new();

                        for j in range(0, v.len()) {
                            if i == j {
                                new_v.push(v[j].clone());
                            } else {
                                new_v.push(v[j].add_in_series(&new_r));
                            }
                        }

                        self.queue.push(Parallel(new_v));
                    }
                }
            }
        }
    }

    fn search(&mut self) -> Option<ResistorTree> {
        for val in self.available.iter() {
            self.queue.push(Resistor(*val));
        }

        while !self.queue.is_empty() {
            let rt = self.queue.pop_front().unwrap();

            if !self.explored.contains(&rt) {
                self.explored.insert(rt.clone());

                if self.is_goal(&rt) {
                    return Some(rt)
                }

                self.explore_node(&rt);
            }
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

            Some(tree)
        },

        None => {
            println!("No arrangement found!");
            None
        }
    }
}

fn freq_search<'a>(target: f64, resistors: &'a Vec<f64>, threshold: f64) {
    println!("To achieve a frequency of {}Hz:", target);

    let high = 0.75 / target;
    let low = 0.25 / target;
    let r1 = (high - low) / (0.6931 * 0.000047);
    let r2 = high / (0.6931 * 0.000047);

    println!("R1 = {}Ω", r1);
    println!("R2 = {}Ω", r2);
    println!("");

    resistor_search(r1, resistors, threshold);
    println!("");

    resistor_search(r2, resistors, threshold);
    println!("");
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
        } else if args[i].as_slice() == "-t" {
            if i + 1 >= args.iter().len() {
                return print_usage()
            }

            threshold = match from_str(args[i + 1].as_slice()) {
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
                resistor_search(target, &resistors, threshold); 
            }
        },

        Some(val) => {
            if val < 0.0 {
                return print_usage();
            } else {
                freq_search(val, &resistors, threshold);
            }
        }
    };
}

