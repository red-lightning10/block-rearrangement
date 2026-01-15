from typing import Any, List, Optional, Tuple
import os
import re
from pyperplan import planner
from pyperplan.planner import SEARCHES, HEURISTICS

class TAMPInterface:
    
    @staticmethod
    def _write_task_file(domain_file: str,
                         grounded_predicates: List[str],
                         goal: str,
                         filename: str,
                         objects: Optional[List[str]] = None) -> None:
        """Write a simple PDDL problem file containing the supplied grounded predicates and goal.
        If `objects` is provided, it will be used for the (:objects ...) clause.
        Otherwise the function heuristically extracts candidate object names
        from the predicates and goal text.
        """
        try:
            with open(domain_file, "r", encoding="utf-8") as df:
                txt = df.read()
            m = re.search(r"\(define\s*\(\s*domain\s+([A-Za-z0-9_\-]+)\s*\)", txt, flags=re.IGNORECASE)
            if m:
                domain_name = m.group(1)
            else:
                raise RuntimeError(f"Could not extract domain name from {domain_file}")
        except FileNotFoundError as e:
            raise RuntimeError(f"Domain file not found: {domain_file}") from e
        except IOError as e:
            raise RuntimeError(f"Failed to read domain file: {domain_file}") from e

        # derive objects if not given
        if objects is None:
            text = " ".join(grounded_predicates) + " " + goal
            tokens = re.findall(r"\b[A-Za-z][A-Za-z0-9_]*\b", text)
            # filter out common predicate words
            # TODO: Make this configurable so new predicates can be added easily
            blacklist = {"on", "ontable", "clear", "holding", "handempty", "and", "or", "not"}
            objects = [t for t in sorted(set(tokens), key=str.lower) if t.lower() not in blacklist]

        with open(filename, "w") as f:
            f.write("(define (problem Generated)\n")
            f.write(f"  (:domain {domain_name})\n")
            if objects:
                f.write("  (:objects ")
                f.write(" ".join(obj.lower() for obj in objects))
                f.write(" - block)\n")
            else:
                f.write("  (:objects )\n")
            f.write("  (:init")
            for pred in grounded_predicates:
                f.write(f" ({pred.lower()})")
            f.write(" )\n")
            f.write(f"  (:goal {goal.lower()})\n")
            f.write(")\n")

    @staticmethod
    def plan_from_grounded(domain_file: str,
                            grounded_predicates: List[str],
                            task_filename: Optional[str] = None,
                            goal: str = "(AND (ON D C) (ON C B) (ON B A))",
                            search: str = "astar",
                            heuristic: str = "hff",
                            objects: Optional[List[str]] = None,
                            ) -> List[Any]:
        """Generate a task file from grounded predicates and call pyperplan to plan.

        Args:
            domain_file: path to the domain PDDL file.
            grounded_predicates: list of grounded predicate strings (without surrounding parentheses).
            goal: PDDL goal expression (e.g. "(AND (ON D C) ...)").
            search: key for SEARCHES (default: 'astar').
            heuristic: key for HEURISTICS (default: 'hff').
            task_filename: optional path to write the task file. If None, a temp file is used and removed after planning.
            objects: optional list of object names to include in the (:objects ...) clause.

        Returns:
            The solution returned by pyperplan (list of Operator objects). If planning fails, returns an empty list.
        """

        # write the task
        TAMPInterface._write_task_file(domain_file, grounded_predicates, goal, task_filename, objects=objects)

        try:
            solution = planner.search_plan(domain_file, task_filename, SEARCHES[search], HEURISTICS[heuristic])
            return solution if solution is not None else []
        except Exception as e:
            raise RuntimeError(f"Planning failed: {e}") from e 

    @staticmethod
    def parse_plan_ops(solution: List[Any]) -> List[Tuple[str, List[str]]]:
        """Parse pyperplan Operator objects into (name, [args]) tuples.

        Example: an operator with op.name="(pick-up A)" becomes ("pick-up", ["A"]).
        """
        parsed = []
        for op in solution:
            s = op.name.strip()
            if s.startswith("(") and s.endswith(")"):
                s = s[1:-1].strip()
            parts = s.split()
            if not parts:
                continue
            parsed.append((parts[0], parts[1:]))
        return parsed