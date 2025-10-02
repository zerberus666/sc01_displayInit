idf_component_register(
  SRCS
    hello_world_main.c
    src/lvglTask.c
    GUI/exampleScreen.c
    GUI/footer.c
    GUI/GUI.c
    GUI/header.c
    GUI/mainPanel.c
    GUI/mainScreen.c
    GUI/signalStrengh.c
    GUI/styles.c
    GUI/clockScreen.c
    GUI/img/Circle.c
    GUI/img/clock.c
    GUI/img/example.c
    GUI/img/LefArrow.c
    GUI/img/Menu.c
    GUI/img/settings.c
    GUI/img/clockFace.c
    GUI/img/hourHand.c
    GUI/img/minHand.c
    GUI/img/secHand.c

  INCLUDE_DIRS
    inc

  PRIV_INCLUDE_DIRS
    inc
    GUI

  REQUIRES
    #ft6x36
)

