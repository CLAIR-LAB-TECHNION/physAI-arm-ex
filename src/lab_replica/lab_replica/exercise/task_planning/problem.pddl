; The order-assembly problem.
;
; :objects  the items on the table, by their names in config/items.yaml
;           (e.g. red-block), all of type item
; :goal     any goal over the domain's predicates, e.g. (in-basket red-block)
;
; The goal console reads this file. It keeps the objects (and adds the items
; added with new-item:), and replaces :init with the current state (at the
; start: every object on the table, the gripper empty) and :goal with the
; goal you type. The rest stays as it is.
;
; PDDL reference: https://planning.wiki/ref/pddl/problem

(define (problem order-assembly-problem)
    (:domain order-assembly-domain)

    ; TODO: the items the robot works with. Start with the blocks.
    (:objects
    )

    (:init
        ; filled in by the goal console: the current state
        (robot-gripper-empty)
    )

    ; TODO: a goal of your own, to test the planner with.
    (:goal
        (and
        )
    )
)
