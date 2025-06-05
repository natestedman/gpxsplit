// Copyright 2025 natesm@gmail.com
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR
// IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

use std::fs::File;
use std::io::BufReader;
use std::mem::take;
use std::path::PathBuf;

use anyhow::anyhow;
use anyhow::Context;
use anyhow::Result;
use clap::Parser;
use geoutils::Location;
use gpx::Gpx;
use gpx::Track;
use gpx::TrackSegment;
use gpx::Waypoint;

macro_rules! ok_or_bail {
    ($expr:expr) => {
        match $expr {
            Ok(val) => val,
            Err(err) => return Some(Err(err)),
        }
    };
}

/// Splits a long GPX file into separate files that won't overload the
/// directions calculations on a Wahoo or other navigation device.
///
/// Note this is only tested on the 2025 Tour Divide GPX file, and it assumes
/// a single track and a single segment within that track.
///
/// If that isn't the case, the program will either exit with an error or just
/// not divide the file up, depending on how the file differs. I didn't know
/// that much about the structure and elements of GPX files and just poked
/// around with a debugger after loading the 2025 Tour Divide file to figure
/// out how to achieve this for that file.
#[derive(Parser)]
struct Arguments {
    /// GPX file to split into smaller files. Resulting files will be written to
    /// the same directory, with numbers appended to the component of the
    /// filename before the file extension.
    gpx: PathBuf,

    /// Number of kilometers to include in each file. The file will be cut off
    /// after the next point that exceeds this number, so each file will be
    /// a bit longer than this number. Each succeeding file will include the
    /// final point from the preceeding file, so that the route is not missing
    /// the directions between those two points.
    km_per_file: f64,
}

fn main() -> Result<()> {
    let arguments = Arguments::parse();

    let file = File::open(&arguments.gpx)?;
    let reader = BufReader::new(file);
    let mut gpx = gpx::read(reader)?;

    let waypoints = take(&mut get_segment(&mut gpx)?.points).into_iter();
    let meters_per_file = arguments.km_per_file * 1000.;

    let basename = arguments
        .gpx
        .with_extension("")
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .to_owned();

    let subsequences = LimitDistance {
        waypoints,
        meters_per_file,
        prev_last: None,
    };

    for (index, subsequence) in subsequences.enumerate() {
        let name = format!("{}_{:02}.gpx", basename, index + 1);
        let output = arguments.gpx.with_file_name(&name);

        // update the GPX with the current set of waypoints, then write it to a numbered file
        get_track(&mut gpx)?.name = Some(name);
        get_segment(&mut gpx)?.points = subsequence?;

        let file = File::create_new(&output)
            .with_context(|| format!("failed to create file {}", output.display()))?;
        gpx::write(&gpx, file)?;
    }

    Ok(())
}

fn get_track(gpx: &mut Gpx) -> Result<&mut Track> {
    Ok(gpx
        .tracks
        .get_mut(0)
        .ok_or_else(|| anyhow!("gpx file missing track 0"))?)
}

fn get_segment(gpx: &mut Gpx) -> Result<&mut TrackSegment> {
    Ok(get_track(gpx)?
        .segments
        .get_mut(0)
        .ok_or_else(|| anyhow!("gpx track 0 missing segment 0"))?)
}

/// Iterator of waypoints that reads from an underlying iterator and yields
/// subsequences of waypoints, each one running until the `meters_per_file`
/// distance has been reached.
struct LimitDistance<Waypoints> {
    waypoints: Waypoints,
    meters_per_file: f64,
    prev_last: Option<Waypoint>,
}

impl<Waypoints: Iterator<Item = Waypoint>> Iterator for LimitDistance<Waypoints> {
    type Item = Result<Vec<Waypoint>>;

    fn next(&mut self) -> Option<Self::Item> {
        let first = self.waypoints.next()?;

        let mut accumulated_meters: f64;
        let mut accumulated_waypoints: Vec<Waypoint>;

        // include the last waypoint from the previous segment so that we don't lose
        // navigation between those two points
        match self.prev_last.take() {
            Some(prev_last) => {
                accumulated_meters = ok_or_bail!(distance(&prev_last, &first));
                accumulated_waypoints = vec![prev_last, first];
            }
            None => {
                // only applies to the first file
                accumulated_meters = 0.;
                accumulated_waypoints = vec![first];
            }
        }

        while let Some(waypoint) = self.waypoints.next() {
            let prev = accumulated_waypoints.last().unwrap();

            accumulated_meters += ok_or_bail!(distance(prev, &waypoint));
            accumulated_waypoints.push(waypoint);

            if accumulated_meters > self.meters_per_file {
                break;
            }
        }

        self.prev_last = accumulated_waypoints.last().cloned();

        Some(Ok(accumulated_waypoints))
    }
}

fn distance(a: &Waypoint, b: &Waypoint) -> Result<f64> {
    location(a)
        .distance_to(&location(b))
        .map(|distance| distance.meters())
        .map_err(|err| anyhow!("{}", err))
}

fn location(waypoint: &Waypoint) -> Location {
    let point = waypoint.point();
    Location::new(point.y(), point.x())
}
