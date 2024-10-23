# A* Algorithm for Vacuum-Cleaning Agent

## Overview

This project implements an A* search algorithm for a vacuum-cleaning agent operating in a 5x5 grid. The agent's goal is to clean dirty squares while minimizing total cost. The agent starts in the lower-left corner, with the top five squares dirty.

## Features

- **Fully Observable Environment**: The agent can see the entire grid.
- **Deterministic**: A clean square remains clean; a dirty square remains dirty unless cleaned.
- **Agent Actions**: `Left`, `Right`, `Up`, `Down`, `Suck`, each incurring a cost of 1.
- **Costs**: Actions cost 1; each dirty square at each step costs 2.

## Instructions

1. **Part A** uses heuristic `h1(n)` (from Homework 2 Q5.3).
2. **Part B** uses heuristic `h2(n)` (from Homework 2 Q5.4).

# vacuum_astar
