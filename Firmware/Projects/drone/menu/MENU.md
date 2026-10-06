# Menu organization

## Inputs

There are 4 encoders (+ with one button each) in the Khaos-V.
From now on, these encoders will be referred to as:
- `MS`: Model Selection
- `DS`: Display Scale
- `P1`: First parameter knob
- `P2`: Second parameter knob

## Model Selection `MS`
`MS` by default is used to switch between 3 modes:
- `A1`: Analog model 1
- `A2`: Analog model 2
- `D`: Digital model
These modes decide which model _is currently receiving_ the inputs from
`P1` and `P2`.
All three models are executed in parallel and are always producing output CVs.

Pressing the `MS` button will open a list menu which lets the user
choose which *digital* model to display, and it will be used when `MS = D`.

## Display Scale `DS`
`DS` by default changes the scale of the *digital* model.
Its main purpose is to ensure that the oscillator stays within the
screen boundaries.
`DS` also affects the scale of the output CVs.
This way, if the oscillator stays within the boundaries on the screen,
it is guaranteed that the output CVs don't clip.

## Parameter knobs `P1` and `P2`
The parameter knobs control the values of the two editable parameters
of the selected model.
Since these are encoders, the parameters are edited only by increments
and not by the "angle" of the encoder
(it doesn't really make sense to talk about the encoder angle).

The encoder parameter values are stored in memory and are added to the two
input CVs to calculate the final parameter value.
