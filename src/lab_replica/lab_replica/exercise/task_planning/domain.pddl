; The order-assembly domain: the robot picks items up and puts them on the
; table or into the basket.
;
; The names below are fixed: the robot runs exactly these three actions, each
; with one item as its parameter, and the goal console accepts goals over
; exactly these four predicates.
;
;   (on-table ?i)          ?i stands on the table
;   (in-basket ?i)         ?i is in the basket
;   (robot-gripping ?i)    the gripper holds ?i
;   (robot-gripper-empty)  the gripper holds nothing
;
; The requirements are those of the goals (and, or, not, imply, forall,
; exists over items), so that UPF accepts any goal typed in the console.
;
; PDDL reference: https://planning.wiki/ref/pddl/domain

(define (domain order-assembly-domain)
    (:requirements :strips :typing :negative-preconditions :disjunctive-preconditions
                   :existential-preconditions :universal-preconditions)
    (:types item)
    (:predicates
        (on-table ?i - item)
        (in-basket ?i - item)
        (robot-gripping ?i - item)
        (robot-gripper-empty)
    )

    ; TODO: the preconditions and effects of the three actions.
    ; Hint: think of what the robot can physically do. It holds at most one
    ; item, and nothing ever takes an item out of the basket. Every effect
    ; that makes a fact true should make the facts that contradict it false.

    (:action pick-up
        :parameters (?i - item)
        :precondition (and
        )
        :effect (and
        )
    )

    (:action put-on-table
        :parameters (?i - item)
        :precondition (and
        )
        :effect (and
        )
    )

    (:action put-in-basket
        :parameters (?i - item)
        :precondition (and
        )
        :effect (and
        )
    )
)
