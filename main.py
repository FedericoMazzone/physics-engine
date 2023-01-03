from __future__ import annotations

import math
import random
import sys

import pygame
import pygame_gui

COLOR_BLACK = (0, 0, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_RED = (255, 0, 0)
COLOR_WHITE = (255, 255, 255)


ENV_ACC = (0, 0)
# ENV_ACC = (0, 0.005)
USER_ACC = 250
NUM_ENTITIES = int(sys.argv[1]) if len(sys.argv) > 1 else 1

ENV_SIZE_X = 700
ENV_SIZE_Y = 550
MENU_SIZE_X = 200
MENU_SIZE_Y = ENV_SIZE_Y
WIN_SIZE_X = ENV_SIZE_X + MENU_SIZE_X
WIN_SIZE_Y = ENV_SIZE_Y
FPS = 60
VISC_FRIC_COEFF = 0.0
KINETIC_DISSIPATION_FACTOR = 0.0


def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


def vectorize_along_directions(value, dir_x, dir_y):
    if dir_y == 0:
        value_y = 0
        value_x = value
    else:
        r = abs(dir_x / dir_y)
        value_y = value / math.sqrt(r ** 2 + 1)
        value_x = r * value_y
    value_x *= sign(dir_x)
    value_y *= sign(dir_y)
    return value_x, value_y


class Entity:

    radius = None
    mass = None
    x = None
    y = None
    dx = None
    dy = None
    color = COLOR_WHITE

    def __init__(self, radius, mass, x, y, dx=0, dy=0) -> None:
        self.radius = radius
        self.mass = mass
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy

    def distance_squared(self, other_entity: Entity) -> float:
        return (other_entity.x - self.x) ** 2 + (other_entity.y - self.y) ** 2

    def is_colliding(self, other_entity: Entity) -> bool:
        return self.distance_squared(other_entity) <= (self.radius + other_entity.radius) ** 2

    def stop(self):
        self.dx = 0
        self.dy = 0

    def get_speed(self):
        return math.sqrt(self.dx ** 2 + self.dy ** 2)

    def get_kinetic_energy(self):
        return 0.5 * self.mass * (self.dx ** 2 + self.dy ** 2)


# def find_collisions(entities: list[Entity]):

#     collisions = []
#     for i in range(len(entities)):
#         for j in range(i):
#             entity1 = entities[i]
#             entity2 = entities[j]
#             if entity1.is_colliding(entity2):
#                 collisions.append((entity1, entity2))

#     return collisions


# Search for collisions only among pairs whose projections on x-axis intersect.
def find_collisions(entities: list[Entity]):

    collisions = []
    active = set()
    active_to_remove = set()

    entities_sorted_x = sorted(
        entities, key=lambda entity: entity.x - entity.radius)

    for entity in entities_sorted_x:
        active_to_remove.clear()
        for active_entity in active:
            if active_entity.x + active_entity.radius < entity.x - entity.radius:
                active_to_remove.add(active_entity)
            elif entity.is_colliding(active_entity):
                collisions.append((entity, active_entity))
        active.difference_update(active_to_remove)
        active.add(entity)

    return collisions


# If two entities intersect, move them apart.
def tunnelling_correction(entity1: Entity, entity2: Entity) -> None:

    speed1 = entity1.get_speed()
    speed2 = entity2.get_speed()
    speed_tot = speed1 + speed2
    if speed_tot == 0:
        corr_coeff1 = 0.5
        corr_coeff2 = 0.5
    else:
        corr_coeff1 = speed1 / (speed1 + speed2)
        corr_coeff2 = speed2 / (speed1 + speed2)

    delta_x = entity1.x - entity2.x
    delta_y = entity1.y - entity2.y
    R = entity1.radius + entity2.radius

    new_delta_x, new_delta_y = vectorize_along_directions(R, delta_x, delta_y)

    delta_delta_x = new_delta_x - delta_x
    delta_delta_y = new_delta_y - delta_y

    entity1.x += corr_coeff1 * delta_delta_x
    entity1.y += corr_coeff1 * delta_delta_y

    entity2.x -= corr_coeff2 * delta_delta_x
    entity2.y -= corr_coeff2 * delta_delta_y


# # Handle collision between two entities.
# def collision_response(entity1: Entity, entity2: Entity) -> None:

#     mass_coeff1 = 2 * entity2.mass / (entity1.mass + entity2.mass)
#     mass_coeff2 = 2 * entity1.mass / (entity1.mass + entity2.mass)

#     delta_dx = entity1.dx - entity2.dx
#     delta_dy = entity1.dy - entity2.dy

#     delta_x = entity1.x - entity2.x
#     delta_y = entity1.y - entity2.y

#     scalar = delta_dx * delta_x + delta_dy * delta_y
#     norm = delta_x * delta_x + delta_y * delta_y
#     if norm == 0:
#         print("ERROR: concentric entities")
#         return

#     entity1.dx -= mass_coeff1 * scalar * delta_x / norm
#     entity1.dy -= mass_coeff1 * scalar * delta_y / norm
#     entity1.dx *= 1 - KINETIC_DISSIPATION_FACTOR
#     entity1.dy *= 1 - KINETIC_DISSIPATION_FACTOR

#     entity2.dx += mass_coeff2 * scalar * delta_x / norm
#     entity2.dy += mass_coeff2 * scalar * delta_y / norm
#     entity2.dx *= 1 - KINETIC_DISSIPATION_FACTOR
#     entity2.dy *= 1 - KINETIC_DISSIPATION_FACTOR


# Handle collision between two entities.
def collision_response(entity1: Entity, entity2: Entity) -> None:

    m1 = entity1.mass
    m2 = entity2.mass

    n_x = entity1.x - entity2.x
    n_y = entity1.y - entity2.y
    n_norm = math.sqrt(n_x ** 2 + n_y ** 2)
    if n_norm == 0:
        print("ERROR: concentric entities")
        return
    un_x = n_x / n_norm
    un_y = n_y / n_norm
    ut_x = -un_y
    ut_y = un_x

    v1_x = entity1.dx
    v1_y = entity1.dy
    v2_x = entity2.dx
    v2_y = entity2.dy
    v1_n = v1_x * un_x + v1_y * un_y
    v2_n = v2_x * un_x + v2_y * un_y
    v1_t = v1_x * ut_x + v1_y * ut_y
    v2_t = v2_x * ut_x + v2_y * ut_y

    K = 1 - KINETIC_DISSIPATION_FACTOR

    # a = m1 * (m1 + m2)
    # b = -m1 * (m1 * v1_n + m2 * v2_n)
    # c = (m1 * v1_n + m2 * v2_n) ** 2 - K * m2 * \
    #     (m1 * (v1_n ** 2) + m2 * (v2_n ** 2))
    # v1final_n_new = (-b + math.sqrt(b ** 2 - a * c)) / a
    # v2final_n_new = m1 * (v1_n - v1final_n_new) / m2 + v2_n

    v1final_n = K * (v1_n * (m1 - m2) + 2 * m2 * v2_n) / (m1 + m2)
    v2final_n = K * (v2_n * (m2 - m1) + 2 * m1 * v1_n) / (m1 + m2)

    v1final_t = v1_t
    v2final_t = v2_t

    v1final_x = v1final_n * un_x + v1final_t * ut_x
    v1final_y = v1final_n * un_y + v1final_t * ut_y
    v2final_x = v2final_n * un_x + v2final_t * ut_x
    v2final_y = v2final_n * un_y + v2final_t * ut_y

    entity1.dx = v1final_x
    entity1.dy = v1final_y
    entity2.dx = v2final_x
    entity2.dy = v2final_y


def compute_viscous_friction(friction_coeff, radius, speed):
    return 6 * math.pi * friction_coeff * radius * speed


# Generate entities
entities = []
while len(entities) < NUM_ENTITIES:
    radius = random.randint(6, 8)
    entity = Entity(
        radius,       # radius
        math.pi * (radius ** 2),       # mass
        random.randint(0, ENV_SIZE_X),
        random.randint(0, ENV_SIZE_Y),
        100 * (random.random() - 0.5),
        100 * (random.random() - 0.5)
    )
    for other_entity in entities:
        if entity.is_colliding(other_entity):
            break
    else:
        entities.append(entity)

# speed = 20
# entities.append(Entity(20, 10, 100, 200, speed, 0))
# entities.append(Entity(40, 5, 300, 200, -speed, 0))
# entities.append(Entity(20, 10, 200, 100, 0, speed))
# entities.append(Entity(40, 5, 200, 300, 0, -speed))
# entities.append(Entity(20, 10, 100, 100, speed, speed))
# entities.append(Entity(40, 5, 300, 300, -speed, -speed))
# entities.append(Entity(20, 10, 100, 300, speed, -speed))
# entities.append(Entity(40, 5, 300, 100, -speed, speed))
# entities.append(Entity(20, 10, 100, 100, speed, speed))
# entities.append(Entity(40, 10, 300, 100, -speed, speed))
# entities.append(Entity(20, 10, 100, 200, speed, 0))
# entities.append(Entity(20, 10, 200, 200, 0, 0))
# entities.append(Entity(20, 10, 240, 200, 0, 0))
# entities.append(
#     Entity(20, 10, 80, 200, speed, 0))
# entities.append(Entity(20, 10, 300, 100, -speed /
#                 math.sqrt(2), speed / math.sqrt(2)))
# entities.append(Entity(20, 10, 300, 300, -speed /
#                 math.sqrt(2), -speed / math.sqrt(2)))


# Graphics
pygame.init()

win = pygame.display.set_mode((WIN_SIZE_X, WIN_SIZE_Y))
pygame.display.set_caption("Test")

env = pygame.Surface((ENV_SIZE_X, ENV_SIZE_Y))
menu = pygame.Surface((MENU_SIZE_X, MENU_SIZE_Y))
menu.fill(COLOR_WHITE)
win.blit(menu, (ENV_SIZE_X, 0))

manager = pygame_gui.UIManager((WIN_SIZE_X, WIN_SIZE_Y))

y = 10
hello_button = pygame_gui.elements.UIButton(
    relative_rect=pygame.Rect(ENV_SIZE_X + 10, y, 180, 30),
    text='Say Hello',
    manager=manager
)
y += 40

friction_slider_text = pygame_gui.elements.UITextBox(
    f"Friction coeff. {VISC_FRIC_COEFF:.2f}",
    relative_rect=pygame.Rect(ENV_SIZE_X + 10, y, 180, 30),
    manager=manager
)
y += 24

friction_slider = pygame_gui.elements.UIHorizontalSlider(
    relative_rect=pygame.Rect(ENV_SIZE_X + 10, y, 180, 30),
    start_value=VISC_FRIC_COEFF,
    value_range=(0.0, 2.0),
    manager=manager,
    click_increment=0.05
)
y += 40

kinetic_dissipation_slider_text = pygame_gui.elements.UITextBox(
    f"Kinetic diss. {KINETIC_DISSIPATION_FACTOR:.2f}",
    relative_rect=pygame.Rect(ENV_SIZE_X + 10, y, 180, 30),
    manager=manager
)
y += 24

kinetic_dissipation_slider = pygame_gui.elements.UIHorizontalSlider(
    relative_rect=pygame.Rect(ENV_SIZE_X + 10, y, 180, 30),
    start_value=KINETIC_DISSIPATION_FACTOR,
    value_range=(0.0, 1.0),
    manager=manager,
    click_increment=0.05
)
y += 40


left_clicked = False
right_clicked = False


clock = pygame.time.Clock()

run = True
while run:

    dt = clock.tick(FPS) / 1000.0

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            run = False

        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            if event.ui_element == hello_button:
                print("Hello World!")

        if event.type == pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
            if event.ui_element == friction_slider:
                VISC_FRIC_COEFF = friction_slider.get_current_value()
                friction_slider_text.set_text(
                    f"Friction coeff. {VISC_FRIC_COEFF:.2f}")
            if event.ui_element == kinetic_dissipation_slider:
                KINETIC_DISSIPATION_FACTOR = kinetic_dissipation_slider.get_current_value()
                kinetic_dissipation_slider_text.set_text(
                    f"Kinetic diss. {KINETIC_DISSIPATION_FACTOR:.2f}")

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                left_clicked = True
            if event.button == 3:
                right_clicked = True

        if event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                left_clicked = False
            if event.button == 3:
                right_clicked = False

        manager.process_events(event)

    manager.update(dt)

    pygame.display.set_caption(f"{int(clock.get_fps())} fps")

    userInput = pygame.key.get_pressed()

    for entity in entities:

        entity: Entity

        # Update arrowkeys-given acceleration
        if userInput[pygame.K_LEFT]:
            entity.dx -= USER_ACC * dt
        if userInput[pygame.K_RIGHT]:
            entity.dx += USER_ACC * dt
        if userInput[pygame.K_UP]:
            entity.dy -= USER_ACC * dt
        if userInput[pygame.K_DOWN]:
            entity.dy += USER_ACC * dt

        # Update click-given acceleration
        if left_clicked or right_clicked:
            mouse_pos = pygame.mouse.get_pos()
            if mouse_pos[0] >= 0 and mouse_pos[0] < ENV_SIZE_X and mouse_pos[1] >= 0 and mouse_pos[1] < ENV_SIZE_Y:
                # grav_acc = 6.67e-11 * entity.mass * \
                #     1e+13 / (delta_x ** 2 + delta_y ** 2)
                grav_acc = USER_ACC
                if right_clicked:
                    grav_acc = -grav_acc
                delta_x = mouse_pos[0] - entity.x
                delta_y = mouse_pos[1] - entity.y
                ddx, ddy = vectorize_along_directions(
                    grav_acc, delta_x, delta_y)
                entity.dx += ddx * dt
                entity.dy += ddy * dt

        # Update environment-given acceleration
        entity.dx += ENV_ACC[0] * dt
        entity.dy += ENV_ACC[1] * dt

        # Update friction-given acceleration
        friction_acc = compute_viscous_friction(
            VISC_FRIC_COEFF, entity.radius, entity.get_speed()) / entity.mass
        ddx, ddy = vectorize_along_directions(
            friction_acc, entity.dx, entity.dy)
        if entity.dx > 0:
            entity.dx = max(0, entity.dx - ddx * dt)
        else:
            entity.dx = min(0, entity.dx - ddx * dt)
        if entity.dy > 0:
            entity.dy = max(0, entity.dy - ddy * dt)
        else:
            entity.dy = min(0, entity.dy - ddy * dt)

        # Update position
        entity.x += entity.dx * dt
        entity.y += entity.dy * dt

        # Check wall collision
        if entity.x < 0 + entity.radius:
            entity.x = 0 + entity.radius
            entity.dx = -entity.dx * (1 - KINETIC_DISSIPATION_FACTOR)
            entity.color = COLOR_RED
        elif entity.x > ENV_SIZE_X - entity.radius:
            entity.x = ENV_SIZE_X - entity.radius
            entity.dx = -entity.dx * (1 - KINETIC_DISSIPATION_FACTOR)
            entity.color = COLOR_RED
        if entity.y < 0 + entity.radius:
            entity.y = 0 + entity.radius
            entity.dy = -entity.dy * (1 - KINETIC_DISSIPATION_FACTOR)
            entity.color = COLOR_RED
        elif entity.y > ENV_SIZE_Y - entity.radius:
            entity.y = ENV_SIZE_Y - entity.radius
            entity.dy = -entity.dy * (1 - KINETIC_DISSIPATION_FACTOR)
            entity.color = COLOR_RED

    # Find collisions between entities
    collisions = find_collisions(entities)

    # Handle collisions
    for entity1, entity2 in collisions:
        entity1.color = COLOR_RED
        entity2.color = COLOR_RED
        tunnelling_correction(entity1, entity2)
        collision_response(entity1, entity2)

    # Update canvas
    env.fill(COLOR_BLACK)
    for entity in entities:
        if entity.color != COLOR_WHITE:
            new_color_value = min(255, entity.color[1] + 10)
            entity.color = (255, new_color_value, new_color_value)
        pygame.draw.circle(
            env, entity.color, (entity.x, entity.y), entity.radius)
    win.blit(env, (0, 0))

    manager.draw_ui(win)

    pygame.display.update()
